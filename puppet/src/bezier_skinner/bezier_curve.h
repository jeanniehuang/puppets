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

#pragma once

#include "mLib/include/ext-flann/nearestNeighborSearchFLANN.h"

#include "bezier_curve_base.h"

/** \file
 * A Bezier curve is a collection of cubic bezier segments (BezierSegment) that also stores
 * continuity constraints between the segments.

 * Adobe patent application tracking P7442-US
 * Title: "Generating A Triangle Mesh for An Image Represented by Curves"
 * Inventors: Vineet Batra, Matthew Fisher, Kevin Wampler, Danny Kaufman, Ankit Phogat

 */

#ifdef WIN_ENV
__pragma(warning(push))
__pragma(warning(disable:4267))
__pragma(warning(disable:4244))
__pragma(warning(disable:4018))
__pragma(warning(disable:4305))
#endif

namespace extensions
{
    namespace puppet
    {
        struct AngleConstraint
        {
            AngleConstraint(int _s0, const BezierSegment& seg0, int _s1, const BezierSegment& seg1)
            {
                s0 = _s0;
                s1 = _s1;
                const geom::Vector2f zero(0.0f,0.0f);
                
                geom::Vector2f s0_vec(seg0.p[3] - seg0.p[2]);
                geom::Vector2f s1_vec(seg1.p[1] - seg1.p[0]);
                if(s0_vec == zero)
                    s0_vec = geom::Vector2f(seg0.p[3] - seg0.p[1]);
                if(s0_vec == zero)
                    s0_vec = geom::Vector2f(seg0.p[3] - seg0.p[0]);
                
                if(s0_vec != zero)
                    s0_vec = s0_vec.unit();
                if(s1_vec == zero)
                    s1_vec = geom::Vector2f(seg1.p[2] - seg1.p[0]);
                if(s1_vec == zero)
                    s1_vec = geom::Vector2f(seg1.p[3] - seg1.p[0]);
                if(s1_vec != zero)
                    s1_vec = s1_vec.unit();
                
                dot_product = (-s0_vec).dot(s1_vec);
                cross_product = (-s0_vec).cross(s1_vec);
            }
            // indices of the segments involved.
            int s0 = -1, s1 = -1;
            float dot_product = 0;
            float cross_product = 0;
            
            // constraint is satisfied if
            // RotationMatrix * (segments[s0].p2 - segments[s0].p3) == segments[s1].p1 - segments[s1].p0
        };
        
        // a list of bezier segments with explicitly encoded continuity
        struct BezierCurve : public BezierCurveBase
        {
            std::vector<AngleConstraint> AngleConstraints;
            
            BezierCurve() = default;
            
            // load bezier curve from SVG file using nanosvg
            /*BezierCurve(const string &SVGFilename, double sampleSpacing, const PuppetMesh<double> &mesh)
             {
             NSVGimage *svgImage;
             svgImage = nsvgParseFromFile(SVGFilename.c_str(), "px", 300);
             const double height = svgImage->height;
             
             // create a BezierSegment for each curve segment in the svg
             for (NSVGshape *shape = svgImage->shapes; shape != NULL; shape = shape->next) {
             for (NSVGpath *path = shape->paths; path != NULL; path = path->next) {
             for (int i = 0; i < path->npts - 1; i += 3) {
             float* p = &path->pts[i * 2];
             
             BezierSegment s;
             s.p[0] = vec2f(p[0], height - p[1]);
             s.p[1] = vec2f(p[2], height - p[3]);
             s.p[2] = vec2f(p[4], height - p[5]);
             s.p[3] = vec2f(p[6], height - p[7]);
             
             segments.push_back(s);
             }
             }
             }
             
             nsvgDelete(svgImage);
             
             // SVG files do not encode C0/C1, so add all implied constraints
             addImplicitConstraints();
             
             // samples bezier curves uniformly at given spacing, and determine mesh UVW coordinates for each sample
             bindSamplesToMesh(sampleSpacing, mesh);
             }*/
            
            void AddSegments(const Eigen::Matrix2Xd& pts)
            {
                //special case where we have just one point
                if (pts.cols() == 1)
                {
                    BezierSegment s;
                    s.p[0] = geom::Vector2f(pts(0, 0), pts(1, 0));
                    s.p[1] = geom::Vector2f(pts(0, 0), pts(1, 0));
                    s.p[2] = geom::Vector2f(pts(0, 0), pts(1, 0));
                    s.p[3] = geom::Vector2f(pts(0, 0), pts(1, 0));
                    
                    segments.push_back(s);
                }
                else
                {
                    for (int i = 0; i < pts.cols() - 1; /*i += 3*/)
                    {
                        BezierSegment s;
                        s.p[0] = geom::Vector2f(pts(0, i), pts(1, i));
                        ++i;
                        s.p[1] = geom::Vector2f(pts(0, i), pts(1, i));
                        ++i;
                        s.p[2] = geom::Vector2f(pts(0, i), pts(1, i));
                        ++i;
                        s.p[3] = geom::Vector2f(pts(0, i), pts(1, i));
                        
                        segments.push_back(s);
                    }
                }
                
                // SVG files do not encode C0/C1, so add all implied constraints
                addImplicitConstraints();
            }
            
#if 0
            BezierCurve(const Eigen::Matrix2Xd& pts, double sampleSpacing, const optimtools::PuppetMesh<double> &mesh)
            {
                AddSegments(pts);
                // samples bezier curves uniformly at given spacing, and determine mesh UVW coordinates for each sample
                bindSamplesToMesh(sampleSpacing, mesh);
            }
#endif
            BezierCurve(const Eigen::Matrix2Xd& pts)
            {
                AddSegments(pts);
            }
            
            float constraintError(const AngleConstraint &c) const
            {
                if(c.dot_product == 0.0f && c.cross_product == 0.0f)
                    return 1.0f;
                return 0;
            }
            
            // projects the curve down into one that satisfies all constraints
            void projectToConstraints()
            {
                for (auto &c : C0Constraints)
                {
                    BezierSegment &s0 = segments[c.s0];
                    BezierSegment &s1 = segments[c.s1];
                    const geom::Vector2f pFinal = (s0.p[3] + s1.p[0]) * 0.5f;
                    s0.p[3] = pFinal;
                    s1.p[0] = pFinal;
                }
                
                for (auto &c : C1Constraints)
                {
                    BezierSegment &s0 = segments[c.s0];
                    BezierSegment &s1 = segments[c.s1];
                    // TODO: make this "fair" so s1.p[1] is also adjusted
                    s0.p[2] = s0.p[3] + s1.p[0] - s1.p[1];
                }
            }
            
            // subdivides the curve one level.
            // TODO: this is currently incomplete and is just used for debugging performance. It should be updated to use actual bezier subdivision.
            BezierCurve subdivide() const
            {
                BezierCurve result;
                for (const BezierSegment &s : segments)
                {
                    BezierSegment s0, s1;
                    s.subdivide(s0, s1);
                    result.segments.push_back(s0);
                    result.segments.push_back(s1);
                }
                for (int i = 0; i < result.segments.size() - 1; i++)
                {
                    result.C0Constraints.push_back(C0Constraint(i, i + 1));
                    result.C1Constraints.push_back(C1Constraint(i, i + 1));
                }
                return result;
            }
            
            // samples bezier curves uniformly at given spacing, and determine mesh UVW coordinates for each sample
            /*void bindSamplesToMesh(double sampleSpacing, const mesh_traits<double, 2>::PointVector &meshPoints, const mesh_traits<double, 2>::TriangleVector &triangles)
             {
             for (auto &s : segments)
             {
             s.initSamples(sampleSpacing);
             s.bindSamplesToMesh(meshPoints, triangles);
             }
             }*/
            void bindSamplesToMesh(double sampleSpacing,
                                   optimtools::PuppetMeshGeometry<double> &accelerator)
            {
                for (auto &s : segments)
                {
                    s.initSamples(sampleSpacing);
                    s.bindSamplesToMesh(accelerator);
                }
            }
            
            // finds all segments that are approximately C0 and C1 continuous, and adds them as C0/C1 constraints
            void addImplicitConstraints()
            {
                const int searchCount = 100;
                const int treeCount = 4;
                const int maxK = 8;
                const double C0Threshold = 1e-2;
                const double C1Threshold = 1e-1;
                const double G1Threshold = 1e-1;
                
                
                // build accelerators for control points P0 and P3 of each segment
                ml::NearestNeighborSearchFLANNf acceleratorP0(searchCount, treeCount);
                //NearestNeighborSearchFLANNf acceleratorP3(searchCount, treeCount);
                
                std::vector<const float *> P0list;
                //vector<const float *> P3list;
                for (auto &s : segments)
                {
                    P0list.push_back((const float *)&s.p[0]);
                    //P3list.push_back((const float *)&s.p[3]);
                }
                
                acceleratorP0.init(P0list, 2, maxK);
                //acceleratorP3.init(P3list, 2, maxK);
                
                for (size_t index = 0; index < segments.size(); index++)
                {
                    const auto& s = segments[index];
                    const std::vector<uint32_t> kNearest = acceleratorP0.fixedRadius((const float *)&s.p[3], maxK, C0Threshold * C0Threshold);
                    for (int n : kNearest)
                    {
                        if (n != index)
                        {
                            C0Constraints.push_back(C0Constraint(index, n));
                            // check if these two curves are also C1 continuous
                            C1Constraint c1Candidate = C1Constraint(index, n);
                            float c1Error = BezierCurveBase::constraintError(c1Candidate);
                            //cout << "C1 error: " << c1Error << endl;
                            if (c1Error < C1Threshold)
                            {
                                C1Constraints.push_back(c1Candidate);
                            }
                            else
                            {
                                G1Constraint g1Candidate = G1Constraint(index, segments[index], n, segments[n]);
                                float g1Error = BezierCurveBase::constraintError(g1Candidate);
                                if (g1Error < G1Threshold)
                                {
                                    G1Constraints.push_back(g1Candidate);
                                }
                                else
                                {
                                    AngleConstraint aCandidate = AngleConstraint(index, segments[index], n, segments[n]);
                                    float aError = constraintError(aCandidate);
                                    if(aError == 0.0f)
                                        AngleConstraints.push_back(aCandidate);
                                }
                            }
                            
                        }
                    }
                }
                //		cout << "implied C0 constraints: " << C0Constraints.size() << endl;
                //		cout << "implied C1 constraints: " << C1Constraints.size() << endl;
            }
        };
        
        namespace
        {
            // loads a debug curve used for testing.
            BezierCurve createDebugCurve(const optimtools::mesh_traits<double, 2>::PointVector &meshPoints,
                                         const optimtools::mesh_traits<double, 2>::TriangleVector &triangles,
                                         int subdivisionCount)
            {
                std::vector<geom::Vector2f> points;
                points.push_back(geom::Vector2f(161.5f, 1182.35f));
                points.push_back(geom::Vector2f(188.45f, 1263.2f));
                points.push_back(geom::Vector2f(292.4f, 1274.75f));
                points.push_back(geom::Vector2f(373.25f, 1190.05f));
                
                points.push_back(geom::Vector2f(369.4f, 1186.2f));
                points.push_back(geom::Vector2f(411.75f, 1105.35f));
                points.push_back(geom::Vector2f(519.55f, 1120.75f));
                points.push_back(geom::Vector2f(565.75f, 1220.85f));
                
                points.push_back(geom::Vector2f(565.75f, 1220.85f));
                points.push_back(geom::Vector2f(662.0f, 1243.95f));
                points.push_back(geom::Vector2f(700.5f, 1347.9f));
                points.push_back(geom::Vector2f(658.15f, 1421.05f));
                
                points.push_back(geom::Vector2f(658.15f, 1421.05f));
                points.push_back(geom::Vector2f(619.65f, 1617.4f));
                points.push_back(geom::Vector2f(654.3f, 1817.6f));
                points.push_back(geom::Vector2f(765.95f, 1802.2f));
                
                BezierCurve curve;
                for (int i = 0; i < points.size(); i += 4)
                {
                    BezierSegment segment;
                    segment.p[0] = points[i + 0];
                    segment.p[1] = points[i + 1];
                    segment.p[2] = points[i + 2];
                    segment.p[3] = points[i + 3];
                    curve.segments.push_back(segment);
                }
                
                curve.C0Constraints.push_back(C0Constraint(0, 1));
                curve.C0Constraints.push_back(C0Constraint(1, 2));
                curve.C0Constraints.push_back(C0Constraint(2, 3));
                
                curve.C1Constraints.push_back(C1Constraint(0, 1));
                curve.C1Constraints.push_back(C1Constraint(2, 3));
                
                for (int i = 0; i < subdivisionCount; i++)
                    curve = curve.subdivide();
                
                //cout << "constraint error: " << curve.constraintError() << endl;
                curve.projectToConstraints();
                //cout << "constraint error: " << curve.constraintError() << endl;
                
                //curve.bindSamplesToMesh(20.0f, meshPoints, triangles);
                
                //cout << "length: " << curve.segments[0].estimateLength() << endl;
                
                return curve;
            }
        }
    }
}

#ifdef WIN_ENV
__pragma(warning(pop))
#endif
