/*************************************************************************
 *
 * ADOBE CONFIDENTIAL
 * ___________________
 *
 *  Copyright 2025 Adobe Systems Incorporated
 *  All Rights Reserved.
 *
 * NOTICE:  All information contained herein is, and remains
 * the property of Adobe Systems Incorporated and its suppliers,
 * if any.  The intellectual and technical concepts contained
 * herein are proprietary to Adobe Systems Incorporated and its
 * suppliers and are protected by trade secret or copyright law.
 * Dissemination of this information or reproduction of this material
 * is strictly forbidden unless prior written permission is obtained
 * from Adobe Systems Incorporated.
 **************************************************************************/

#include "graphics/Graphics"
#include "base/threads/Threads"

#include "bezier_skinner/bezier_curve.h"
#include "bezier_skinner/skinner_LSQ.h"

#include "PuppetTypes.h"
#include "PuppetVectorSkinner.h"

namespace extensions
{
    namespace puppet
    {
        USING_OPTIMTOOLS_MESH_TYPES(typename,Scalar,2);

        // Visitor that converts sketchlib FigureGraphic instances to BezierCurve instances
        // Creates new BezierCurve instances for each FigureGraphic encountered
        class ToBezierCurveConverter : public graphics::ConstDFVisitor {
            std::vector<BezierCurve>& _bzCurves;
        public:
            explicit ToBezierCurveConverter(std::vector<BezierCurve>& bzCurves) :
                _bzCurves(bzCurves)
            {
            }

            void beginGraphic(const graphics::Group& g) {}
            void endGraphic(const graphics::Group& g) {}
            void visitGraphic(const graphics::ImageGraphic& g) {}
            
            void visitGraphic(const graphics::FigureGraphic& g)
            {
                const shapes::Path::Curves& curves = g.figure().path().curves;
                for (const auto& curve : curves)
                {
                    _bzCurves.push_back(BezierCurve());
                    BezierCurve& bzCurve = _bzCurves.back();
                    
                    const geom::BezierSpline2f::Points& points = curve.pts;
                    const size_t numPts = points.size();
                    if (curve.degree == 1)
                    {
                        for (size_t i=1; i< numPts; i++)
                        {
                            BezierSegment segment;
                            segment.p[0] = segment.p[1] = points[i-1];
                            segment.p[2] = segment.p[3] = points[i];
                            
                            bzCurve.segments.push_back(segment);
                        }
                    }
                    else if (curve.degree == 3)
                    {
                        for (size_t i=1; i< numPts; i += 3)
                        {
                            BezierSegment segment;
                            segment.p[0] = points[i-1];
                            segment.p[1] = points[i];
                            segment.p[2] = points[i+1];
                            segment.p[3] = points[i+2];
                            
                            bzCurve.segments.push_back(segment);
                        }
                    }
                }
            }
        };
    
        // Visitor that converts BezierCurve instances back to sketchlib FigureGraphic instances
        // Assumes that bzCurves and skinnedBzCurves have the same size. Also assumes that
        // there already is a 1-to-1 mapping from the FigureGraphic's encountered
        // to the BezierCurve's that exist in the provided vectors.
        class FromBezierCurveConverter : public graphics::DFVisitor {
            const std::vector<BezierCurve>& _bzCurves;
            const std::vector<BezierCurve>& _skinnedBzCurves;
            size_t _currentIndex;
        public:
            explicit FromBezierCurveConverter(const std::vector<BezierCurve>& bzCurves,
                                              const std::vector<BezierCurve>& skinnedBzCurves) :
                _bzCurves(bzCurves),
                _skinnedBzCurves(skinnedBzCurves),
                _currentIndex(0)
            {
            }

            void beginGraphic(graphics::Group& g) {}
            void endGraphic(graphics::Group& g) {}
            void visitGraphic(graphics::ImageGraphic& g) {}
            
            void visitGraphic(graphics::FigureGraphic& g)
            {
                shapes::Path::Curves& curves = g.mutableFigure().mutablePath().curves;
                for (auto& curve : curves)
                {
                    // Ran out of _skinnedBzCurves? This is not expected
                    if (_currentIndex >= _skinnedBzCurves.size())
                        break;
                    const BezierCurve& bzCurve = _bzCurves[_currentIndex];
                    const BezierCurve& skinnedBzCurve = _skinnedBzCurves[_currentIndex];
                    const size_t numSegments = skinnedBzCurve.segments.size();
                    _currentIndex += 1;

                    // Calculate the required number of points needed in Path::Curve
                    // to support the number of segments
                    const size_t requiredNumPoints = numSegments * 3 + 1;
                    
                    geom::BezierSpline2f::Points& points = curve.pts;
                    if (curve.degree != 3
                        || points.size() != requiredNumPoints
                        || bzCurve.segments.size() != numSegments)
                    {
                        // If degree != 3, or the number of points don't meet the
                        // requirement, or the number of segments differ between
                        // the original and skinned curves, then simply set the degree to 3,
                        // clear the points, and re-populate with the skinned segments
                        curve.degree = 3;
                        points.clear();

                        points.reserve(requiredNumPoints);
                        
                        for (size_t i=0; i< numSegments; i++)
                        {
                            const BezierSegment& segment = skinnedBzCurve.segments[i];
                            
                            // Record first point of first segment. Subsequent segments
                            // will have redundant first points.
                            if (i == 0)
                                points.push_back(segment.p[0]);
                            
                            points.push_back(segment.p[1]);
                            points.push_back(segment.p[2]);
                            points.push_back(segment.p[3]);
                        }
                    }
                    else
                    {
                        // If degree == 3, then try to maintain C1 continuity where appropriate
                        size_t pointsIndex = 0;
                        for (size_t i=0; i< numSegments; i++)
                        {
                            const BezierSegment& segment = bzCurve.segments[i];
                            const BezierSegment& skinnedSegment = skinnedBzCurve.segments[i];
                            if (i == 0)
                            {
                                // Record first two points of first segment
                                points[0] = skinnedSegment.p[0];
                                points[1] = skinnedSegment.p[1];
                            }
                            
                            bool needsContinuityAdjustment = false;
                            
                            // Grab next segment, or first segment if at the end
                            bool atEnd = (i == numSegments-1);
                            const BezierSegment& nextSeg = atEnd ? bzCurve.segments[0]
                                                                 : bzCurve.segments[i+1];
                            const BezierSegment& nextSkinnedSeg = atEnd ? skinnedBzCurve.segments[0]
                                                                        : skinnedBzCurve.segments[i+1];
                            
                            // If we're at the end, wrap around back to the beginning
                            // of the points vector, assuming a closed path.
                            size_t outPointIndex = atEnd ? 0 : pointsIndex + 3;

                            // Ensure that the current segment's end point is the same as
                            // the next segment's beginning point
                            if (equalWithinTol(skinnedSegment.p[3], nextSkinnedSeg.p[0]))
                            {
                                // Check if the original spline is continuous at the given point
                                needsContinuityAdjustment = checkIfContinuous(segment.p[2],
                                                                              nextSeg.p[1],
                                                                              segment.p[3]);
                            }
                            
                            // Record the in-handle and final point from the skinned segment
                            points[pointsIndex+2] = skinnedSegment.p[2];
                            points[pointsIndex+3] = skinnedSegment.p[3];
                            
                            // When not at the end, record the out-handle from the next skinned segment
                            if (!atEnd)
                                points[pointsIndex+4] = nextSkinnedSeg.p[1];

                            if (needsContinuityAdjustment)
                            {
                                // Adjust the tangents to enforce C1 continuity
                                adjustTangentsForContinuity(points[pointsIndex+2],
                                                            points[outPointIndex+1],
                                                            points[pointsIndex+3]);
                            }

                            pointsIndex += 3;
                        }
                    }
                }
            }
            
            // Helper method that determines whether the tangents at the given point
            // on a Bezier spline are C1 continuous.
            bool checkIfContinuous(const geom::Vector2f& inHandle,
                                   const geom::Vector2f& outHandle,
                                   const geom::Vector2f& point)
            {
                bool continuous = false;
                
                // Calculate vector for in-handle
                geom::Vector2f vecIn = inHandle - point;
                float lenIn = vecIn.norm2();

                // Calculate vector for out-handle
                geom::Vector2f vecOut = outHandle - point;
                float lenOut = vecOut.norm2();

                // If either the in- or out-handles are on the actual segment point,
                // then continuity is considered false.
                if ( !closeToZero(lenIn) && !closeToZero(lenOut) )
                {
                    // Convert to unit vectors
                    vecIn /= lenIn;
                    vecOut /= lenOut;

                    // Calculate what the out-handle would need to be in order to ensure C1 continuity
                    geom::Vector2f newOut = (-vecIn) * lenOut + point;

                    // Check if this is reasonably close to what the out-handle currently is.
                    // If so, then continuity is achieved.
                    continuous = equalWithinTol(newOut, outHandle);
                }
                return continuous;
            }
            
            // Helper method that adjusts the inHandle and outHandle tangents in order to achieve
            // C1 continuity at the given point. This method averages the difference between the two
            // tangents in order to achieve continuity.
            void adjustTangentsForContinuity(geom::Vector2f& inHandle,
                                             geom::Vector2f& outHandle,
                                             const geom::Vector2f& point)
            {
                // Calculate vector for in-handle
                geom::Vector2f vecIn = inHandle - point;
                float lenIn = vecIn.norm2();

                // Calculate vector for out-handle
                geom::Vector2f vecOut = outHandle - point;
                float lenOut = vecOut.norm2();

                // If either the in- or out-handles are on the actual segment point,
                // then there's no need to fix this up.
                if ( !closeToZero(lenIn) && !closeToZero(lenOut) )
                {
                    // Convert to unit vectors
                    vecIn /= lenIn;
                    vecOut /= lenOut;

                    // Calculate what the out-handle would need to be in order to ensure C1 continuity
                    geom::Vector2f newOut = (-vecIn) * lenOut + point;
                    
                    // Calculate the difference between the ideal out-handle and the current out-handle
                    geom::Vector2f diff = newOut - outHandle;
                    float diffLen = diff.norm2();
                    
                    // Ensure the difference is great enough to warrant changing but not too
                    // big to alter the look of the curve in a significant way
                    constexpr float maxDiff = 2.0f;
                    if (!closeToZero(diffLen) && diffLen < maxDiff)
                    {
                        // Modify out-handle so that it averages the difference between the two handles
                        newOut = (outHandle + newOut) / 2.0;
                        vecOut = newOut - point;
                        lenOut = vecOut.norm2();
                        if (!closeToZero(lenOut))
                        {
                            vecOut /= lenOut;
                            
                            // Now recalculate the in-handle based on the new out handle
                            geom::Vector2f newIn = (-vecOut) * lenIn + point;
                            
                            // Assign back to handles
                            inHandle = newIn;
                            outHandle = newOut;
                        }
                    }
                }
            }
            
            inline bool equalWithinTol(const geom::Vector2f& a, const geom::Vector2f& b)
            {
                constexpr double tolerance = 0.001;
                return (std::abs(a.x - b.x) < tolerance
                        && std::abs(a.y - b.y) < tolerance);
            }
            
            inline bool closeToZero(float value)
            {
                constexpr double tolerance = 0.001;
                return (std::abs(value) < tolerance);
            }
        };


        class ConcretePuppetVectorSkinner : public PuppetVectorSkinner
        {
            // Holds original BezierCurve's and deformed BezierCurve's
            std::shared_ptr<graphics::Group> _group;
            std::vector<BezierCurve> _bzCurves;
            std::vector<BezierCurve> _skinnedBzCurves;
            
            // Holds the previous mesh points for optimization
            std::vector<std::shared_ptr<PointVector>> _prevMeshPoints;

            // Holds an lsq system instance
            SkinnerLSQ _skinnerLSQ;
            
            // Holds a job pool for parallelizing the work to run thru
            // the Bezier curves
            std::shared_ptr<base::JobPool> _jobPool;
            
        public:
            ConcretePuppetVectorSkinner()
            {
                // Share the job pool among all instances
                static std::weak_ptr<base::JobPool> sJobPool;
                _jobPool = sJobPool.lock();
                if (!_jobPool)
                {
                    _jobPool = base::JobPool::createJobPool("Vector Skinner Job Pool");
                    sJobPool = _jobPool;
                }
            }
            
            void initialize(graphics::GroupRef rootGroup,
                            const std::vector<PuppetMeshRef>& puppetMeshes,
                            const base::Progress& progress) override
            {
                _group = rootGroup;
                
                importFiguresToBezierCurves();
                bindSamplesToMesh(puppetMeshes, progress);
                
                if (false)  // skip using the LSQ system for now
                    setupLSQSystem();
                
                // Update the prev mesh points
                _prevMeshPoints.resize(puppetMeshes.size());
                for (int i=0; i< puppetMeshes.size(); i++)
                    _prevMeshPoints[i] = puppetMeshes[i]->points_ptr();
            }
            
            // Run the SkinnerLSQ logic to deform the Bezier curves, then export the results
            // back to their corresponding sketchlib FigureGraphic instances.
            void applyPuppets(const std::vector<PuppetMeshRef>& puppetMeshes) override
            {
                // Calculate the updated triangles vector
                std::vector<std::vector<bool>> updatedTriangles = getUpdatedTriangles(puppetMeshes);
                
                _skinnerLSQ.skinCurve(_bzCurves,
                                      puppetMeshes,
                                      updatedTriangles,
                                      _skinnedBzCurves,
                                      0, //SkinnerLSQ::kUseCurveFitting,
                                      *_jobPool);
                
                exportBezierCurvesToFigures();
                
                // Update the prev mesh points
                _prevMeshPoints.resize(puppetMeshes.size());
                for (int i=0; i< puppetMeshes.size(); i++)
                    _prevMeshPoints[i] = puppetMeshes[i]->points_ptr();
            }
            
            graphics::GroupRef outputVectors() const override
            {
                return _group;
            }

        private:
            void importFiguresToBezierCurves()
            {
                _bzCurves.clear();
                _skinnedBzCurves.clear();
                
                // Iterate thru group and convert Figures to BezierCurves
                ToBezierCurveConverter converter(_bzCurves);
                graphics::ConstBFTraverseAll traversal;
                graphics::ConstAllDFEnumerator enumerator(traversal, converter);
                _group->visit(enumerator);
                
                // Set output vectors to be identical to input at first
                _skinnedBzCurves = _bzCurves;
            }
            
            // Bind the sample points from the Bezier curves to their corresponding meshes
            void bindSamplesToMesh(const std::vector<PuppetMeshRef>& puppetMeshes,
                                   const base::Progress& progress)
            {
                const float kSampleSpacing = 20.0f;
                
                optimtools::PuppetMeshGeometryBVH<Scalar> accelerator(puppetMeshes);
                accelerator.build_accelerator();
                
                _jobPool->parallel_for(size_t(0), _bzCurves.size(), [&](size_t curve_index)
                {
                    if (progress.isCancelled())
                        return;
                    
                    _bzCurves[curve_index].bindSamplesToMesh(kSampleSpacing, accelerator);
                });
                
                progress.throwIfCancelled();
            }
            
            // Set up LSQ system
            void setupLSQSystem()
            {
                _skinnerLSQ.setupSystem(_bzCurves, SkinnerLSQ::Params(), *_jobPool);
            }
            
            void exportBezierCurvesToFigures()
            {
                // Iterate thru bzCurves and convert BezierCurves to Figures
                FromBezierCurveConverter converter(_bzCurves, _skinnedBzCurves);
                graphics::BFTraverseAll traversal;
                graphics::AllDFEnumerator enumerator(traversal, converter);
                _group->visit(enumerator);
            }
            
            // Helper method to retrieve the vector of updated triangles based on the puppet meshes.
            std::vector<std::vector<bool>> getUpdatedTriangles(const std::vector<PuppetMeshRef>& puppetMeshes)
            {
                // Set up the updatedTriangles 2D vector
                std::vector<std::vector<bool>> updatedTriangles;
                for (int i=0; i< puppetMeshes.size(); i++)
                {
                    std::vector<bool> triIndices(puppetMeshes[i]->triangles().size(), false);
                    updatedTriangles.push_back(std::move(triIndices));
                }

                // Iterate to find what triangles were invalidated
                for (int i=0; i< puppetMeshes.size(); i++)
                {
                    // We assume puppetMeshes and _prevMeshPoints have the same size
                    if (i >= _prevMeshPoints.size())
                        continue;
                    
                    const std::shared_ptr<PointVector>& currPts = puppetMeshes[i]->points_ptr();
                    std::shared_ptr<PointVector> prevPts = _prevMeshPoints[i];
                    
                    // We assume the current and previous point vectors have the same size
                    if (currPts->size() != prevPts->size())
                        continue;
                    
                    // Iterate triangles
                    const TriangleVector& triangles = puppetMeshes[i]->triangles();
                    for (int j=0; j< triangles.size(); j++)
                    {
                        const Triangle& tri = triangles[j];

                        // Compare the previous points with the current points
                        const Point& currPt1 = (*currPts)[tri[0]];
                        const Point& currPt2 = (*currPts)[tri[1]];
                        const Point& currPt3 = (*currPts)[tri[2]];

                        const Point& prevPt1 = (*prevPts)[tri[0]];
                        const Point& prevPt2 = (*prevPts)[tri[1]];
                        const Point& prevPt3 = (*prevPts)[tri[2]];

                        if (!equalWithinTol(currPt1, prevPt1)
                            || !equalWithinTol(currPt2, prevPt2)
                            || !equalWithinTol(currPt3, prevPt3))
                        {
                            updatedTriangles[i][j] = true;
                        }
                    }
                }
                return updatedTriangles;
            }

            inline bool equalWithinTol(const Point& a, const Point& b)
            {
                constexpr double tolerance = 0.001;
                return (std::abs(a.x() - b.x()) < tolerance
                        && std::abs(a.y() - b.y()) < tolerance);
            }
        };
    
        std::shared_ptr<PuppetVectorSkinner> PuppetVectorSkinner::create()
        {
            return std::make_shared<ConcretePuppetVectorSkinner>();
        }
    }
}
