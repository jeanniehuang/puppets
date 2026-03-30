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

#include "optimtools/threading.h"

#include "../PuppetTypes.h"

#include "bezier_curve.h"
#include "bezier_curve_fitter.h"

/** \file
* a bezier skinner that uses a discretized sample point + C0 & C1 constraints in a least-squares solve.
*/

//	Adobe patent application tracking P7419-US,
//	entitled "Digital Media Environment for Intuitive Modifications of Digital Graphics", previously titled "Fast, robust and accurate method for refitting cubic Bezier curves to a set of sample points, subject to constraints, post skinning transformation"
//	inventors Vineet Batra, Ankit Phogat, Mridul Kavidayal, Danny Kaufman, Matthew Fisher

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
        class SkinnerLSQ
        {
        public:
            
            enum SkinningOptions
            {
                kUseCurveFitting  = ((int) (1 << 0)),
                kFastCurveFitting = ((int) (1 << 1))
            };
            
            struct Params {
                // strength of the target sample constraints
                double targetSampleWeight = 1.0;
                
                // strength of enforcing C0 and C1 constraints
                double C0Weight = 10000.0;
                double C1Weight = 10000.0;
                double G1Weight = 10000.0;
            };
            
            // builds the A matrix and factorizes it. Only needs to be called once per curve.
            void setupSystem(const std::vector<BezierCurve> &curves, const Params &params, base::JobPool& jobPool);
            
            // skins the given base curve after deformation by the given mesh vertices and triangles.
            // baseCurve must already have its samples bound to the tri-UVW coordiantes of the given mesh.
            // assumes the outCurves are already populated and contain as many elements as baseCurves.
            // only the baseCurves that are affected by the updated triangles will have their
            // corresponding outCurves updated.
            // updatedTriangles is a 2D vector, where the first index is the puppet index, and the
            // second index is the triangle index within that puppet. The entries are boolean values
            // indicating whether that triangle was updated since the last call to the skinner.
            void skinCurve(const std::vector<BezierCurve> &baseCurves,
                           const std::vector<std::shared_ptr<optimtools::PuppetMesh<Scalar>>> &puppetMeshes,
                           const std::vector<std::vector<bool>> &updatedTriangles,
                           std::vector<BezierCurve> &outCurves,
                           int skinningOptions,
                           base::JobPool& jobPool);
            
        private:
            std::vector <std::shared_ptr<LSQSystem>> m_systems;
            Params m_params;
        };
        
        void SkinnerLSQ::skinCurve(const std::vector<BezierCurve> &baseCurves,
                                   const std::vector<std::shared_ptr<optimtools::PuppetMesh<Scalar>>> &puppetMeshes,
                                   const std::vector<std::vector<bool>> &updatedTriangles,
                                   std::vector<BezierCurve> &outCurves,
                                   int skinningOptions,
                                   base::JobPool& jobPool)
        {
            outCurves.resize(baseCurves.size());
            jobPool.parallel_for(0, baseCurves.size(), [&](size_t i)
            {
                const BezierCurve& baseCurve = baseCurves[i];
                
                // Check if this curve is affected by the updated triangles
                bool updateCurve = false;
                for (int segmentIndex = 0;
                     segmentIndex < baseCurve.segments.size() && !updateCurve;
                     segmentIndex++)
                {
                    const auto &segment = baseCurve.segments[segmentIndex];
                    for (const auto &sample : segment.samples)
                    {
                        int64_t puppetIndex = sample.l.puppet_index;
                        int64_t triIndex = sample.l.tri;
                        if (updatedTriangles[puppetIndex][triIndex])
                        {
                            updateCurve = true;
                            break;
                        }
                    }
                }
                
                // Return early if none of the triangles that this curve passes thru have been updated.
                if (!updateCurve)
                    return;
                
#if 1   // No reliance on LSQ system; just deforms the samples and fits a curved on the relocated points
        // Clients are responsible for ensuring continuity
                
                BezierCurve skinnedCurve;
                skinnedCurve.segments.reserve(baseCurve.segments.size());
                for (int segmentIndex = 0; segmentIndex < baseCurve.segments.size(); segmentIndex++)
                {
                    const auto &segment = baseCurve.segments[segmentIndex];
                    std::vector<geom::Vector2f> curve_samples;
                    curve_samples.reserve(segment.samples.size());
                    for (auto &sample : segment.samples)
                    {
                        const auto& meshPoints = puppetMeshes[sample.l.puppet_index]->points();
                        const auto& triangles = puppetMeshes[sample.l.puppet_index]->triangles();
                        
                        // compute the target position on the deformed mesh
                        const geom::Vector2f samplePt = sample.l.getPosition(meshPoints, triangles);
                        curve_samples.emplace_back(samplePt);
                    }
                    
                    curve_samples.erase(std::unique(curve_samples.begin(), curve_samples.end(),
                                                    [](const geom::Vector2f& l, const geom::Vector2f& r)
                    {
                        return std::abs(l.x - r.x) < std::numeric_limits<float>::epsilon() &&
                               std::abs(l.y - r.y) < std::numeric_limits<float>::epsilon();
                    }),
                                        curve_samples.end());

                    std::vector<geom::Vector2f> curve_points = curve_samples.size() == 1 ? std::vector<geom::Vector2f>(4, curve_samples[0]) :
                                                                                           BezierRep::FitCurve(curve_samples, 2.0);
                    
                    for (size_t curve_index = 0; curve_index < curve_points.size()-1; curve_index += 3)
                    {
                        BezierSegment skinnedSegment;
                        skinnedSegment.p[0] = curve_points[curve_index];
                        skinnedSegment.p[1] = curve_points[curve_index+1];
                        skinnedSegment.p[2] = curve_points[curve_index+2];
                        skinnedSegment.p[3] = curve_points[curve_index+3];
                        
                        skinnedCurve.segments.push_back(skinnedSegment);
                    }
                }
                outCurves[i] = std::move(skinnedCurve);
#else
                // setup b, which is zero except for the target sample constraints
                int bIndex = 0;
                for (int segmentIndex = 0; segmentIndex < baseCurve.segments.size(); segmentIndex++)
                {
                    const auto &segment = baseCurve.segments[segmentIndex];
                    if (skinningOptions & kUseCurveFitting)
                    {
                        std::vector<geom::Vector2f> curve_samples;
                        curve_samples.reserve(segment.samples.size());
                        for (auto &sample : segment.samples)
                        {
                            const auto& meshPoints = puppetMeshes[sample.l.puppet_index]->points();
                            const auto& triangles = puppetMeshes[sample.l.puppet_index]->triangles();
                            
                            // compute the target position on the deformed mesh
                            const geom::Vector2f samplePt = sample.l.getPosition(meshPoints, triangles);
                            
                            geom::Vector2f pt(samplePt(0), samplePt(1));
                            curve_samples.emplace_back(pt);
                        }
                            
                        curve_samples.erase(std::unique(curve_samples.begin(), curve_samples.end(),
                                                        [](const geom::Vector2f& l, const geom::Vector2f& r)
                        {
                            return std::abs(l.x - r.x) < std::numeric_limits<float>::epsilon() &&
                                   std::abs(l.y - r.y) < std::numeric_limits<float>::epsilon();
                        }),
                                            curve_samples.end());
                        
                        double curveFitError = skinningOptions & kFastCurveFitting ?   1.0 : 1.0;
                        std::vector<geom::Vector2f> curve_points = curve_samples.size() == 1 ? std::vector<geom::Vector2f>(4, curve_samples[0]) :
                                                                                               BezierRep::FitCurve(curve_samples, curveFitError);
                        const geom::Vector2f& p0 = curve_points[0];
                        const geom::Vector2f& p1 = curve_points[1];
                        const geom::Vector2f& p2 = curve_points[2];
                        const geom::Vector2f& p3 = curve_points[3];
                        
                        for (auto &sample : segment.samples)
                        {
                            const float t = sample.t;
                            const float u = 1.0f - t;
                            const geom::Vector2f samplePt(u * u * u * p0.x + 3.0 * u * u * t * p1.x + 3.0 * u * t * t * p2.x + t * t * t * p3.x,
                                                          u * u * u * p0.y + 3.0 * u * u * t * p1.y + 3.0 * u * t * t * p2.y + t * t * t * p3.y);
                            
                            for (int dimension = 0; dimension < 2; dimension++)
                            {
                                m_systems[i]->b()[bIndex++] = samplePt(dimension);
                            }
                        }
                    }
                    else
                    {
                        for (auto &sample : segment.samples)
                        {
                            const auto& meshPoints = puppetMeshes[sample.l.puppet_index]->points();
                            const auto& triangles = puppetMeshes[sample.l.puppet_index]->triangles();
                            
                            // compute the target position on the deformed mesh
                            const geom::Vector2f samplePt = sample.l.getPosition(meshPoints, triangles);
                            
                            for (int dimension = 0; dimension < 2; dimension++)
                            {
                                m_systems[i]->b()[bIndex++] = samplePt(dimension);
                            }
                        }
                    }
                }
                
                // solve for the new control points
                Eigen::VectorXd controlPointsOut = m_systems[i]->solve();
                
                // create the skinned curve and load its control points from the computed vector.
                BezierCurve skinnedCurve = baseCurve;
                
                int xIndex = 0;
                for (int segmentIndex = 0; segmentIndex < skinnedCurve.segments.size(); segmentIndex++)
                {
                    auto &segment = skinnedCurve.segments[segmentIndex];
                    for (int controlPointIndex = 0; controlPointIndex < 4; controlPointIndex++)
                        for (int dimension = 0; dimension < 2; dimension++)
                        {
                            segment.p[controlPointIndex](dimension) = controlPointsOut(xIndex++);
                        }
                }
                outCurves[i] = std::move(skinnedCurve);
#endif
            });
        }
        
        void SkinnerLSQ::setupSystem(const std::vector<BezierCurve> &curves, const Params &params, base::JobPool& jobPool)
        {
//		    std::cout << "[SkinnerLSQ]: setupSystem" << std::endl;
            m_params = params;
            m_systems.clear();
            m_systems.resize(curves.size());
            
            jobPool.parallel_for(size_t(0), curves.size(), [&](size_t i) {
                const BezierCurve& curve = curves[i];
                std::shared_ptr<LSQSystem> system = std::make_shared<LSQSystem>();
                // 8 unknowns per segment
                system->reset(curve.segments.size() * 8);
                
                auto getUnknownIndex = [](int segmentIndex, int controlPointIndex, int dimension)
                {
                    return 8 * segmentIndex + 2 * controlPointIndex + dimension;
                };
                
                // target sample constraints: 2 per sample
                for (int segmentIndex = 0; segmentIndex < curve.segments.size(); segmentIndex++)
                {
                    const auto &segment = curve.segments[segmentIndex];
                    for (auto &sample : segment.samples)
                    {
                        const float t = sample.t;
                        const float u = 1.0f - t;
                        //u * u * u * p[0] +
                        //3.0f * u * u * t  * p[1] +
                        //3.0f * u * t  * t  * p[2] +
                        //t  * t  * t  * p[3]
                        for (int dimension = 0; dimension < 2; dimension++)
                        {
                            std::vector< std::pair<int, double> > terms;
                            terms.reserve(4);
                            terms.emplace_back(std::make_pair<int, double>(getUnknownIndex(segmentIndex, 0, dimension), u * u * u));
                            terms.emplace_back(std::make_pair<int, double>(getUnknownIndex(segmentIndex, 1, dimension), 3.0 * u * u * t));
                            terms.emplace_back(std::make_pair<int, double>(getUnknownIndex(segmentIndex, 2, dimension), 3.0 * u * t * t));
                            terms.emplace_back(std::make_pair<int, double>(getUnknownIndex(segmentIndex, 3, dimension), t * t * t));
                            
                            system->addConstraint(terms, m_params.targetSampleWeight);
                        }
                    }
                }
                
                //
                // C0 constraints
                // segments[s0].p3 - segments[s1].p0 == 0
                //
                for (auto &constraint : curve.C0Constraints)
                {
                    for (int dimension = 0; dimension < 2; dimension++)
                    {
                        std::vector< std::pair<int, double> > terms;
                        terms.reserve(2);
                        terms.emplace_back(std::make_pair<int, double>(getUnknownIndex(constraint.s0, 3, dimension), 1.0));
                        terms.emplace_back(std::make_pair<int, double>(getUnknownIndex(constraint.s1, 0, dimension), -1.0));
                        
                        system->addConstraint(terms, m_params.C0Weight);
                    }
                }
                //
                // C1 constraints
                // segments[s0].p3 - segments[s0].p2 == segments[s1].p1 - segments[s1].p0
                // segments[s0].p3 - segments[s0].p2 + segments[s1].p0 - segments[s1].p1 == 0
                //
                for (auto &constraint : curve.C1Constraints)
                {
                    for (int dimension = 0; dimension < 2; dimension++)
                    {
                        std::vector< std::pair<int, double> > terms;
                        terms.reserve(4);
                        terms.emplace_back(std::make_pair<int, double>(getUnknownIndex(constraint.s0, 3, dimension), 1.0));
                        terms.emplace_back(std::make_pair<int, double>(getUnknownIndex(constraint.s0, 2, dimension), -1.0));
                        terms.emplace_back(std::make_pair<int, double>(getUnknownIndex(constraint.s1, 0, dimension), 1.0));
                        terms.emplace_back(std::make_pair<int, double>(getUnknownIndex(constraint.s1, 1, dimension), -1.0));
                        
                        system->addConstraint(terms, m_params.C1Weight);
                    }
                }
                
                for (auto &constraint : curve.G1Constraints)
                {
                    for (int dimension = 0; dimension < 2; dimension++)
                    {
                        std::vector< std::pair<int, double> > terms;
                        terms.reserve(4);
                        terms.emplace_back(std::make_pair<int, double>(getUnknownIndex(constraint.s0, 3, dimension), static_cast<double>(constraint.len1)));
                        terms.emplace_back(std::make_pair<int, double>(getUnknownIndex(constraint.s0, 2, dimension), static_cast<double>(-constraint.len1)));
                        terms.emplace_back(std::make_pair<int, double>(getUnknownIndex(constraint.s1, 0, dimension), static_cast<double>(constraint.len0)));
                        terms.emplace_back(std::make_pair<int, double>(getUnknownIndex(constraint.s1, 1, dimension), static_cast<double>(-constraint.len0)));
                        
                        system->addConstraint(terms, m_params.G1Weight);
                    }
                }
#if 0
                for (auto &constraint : curve.AngleConstraints)
                {
                    {
                        vector< pair<int, double> > terms;
                        terms.reserve(6);
                        terms.emplace_back(std::make_pair<int, double>(getUnknownIndex(constraint.s0, 3, 0), -constraint.dot_product));
                        terms.emplace_back(std::make_pair<int, double>(getUnknownIndex(constraint.s0, 2, 0),  constraint.dot_product));
                        terms.emplace_back(std::make_pair<int, double>(getUnknownIndex(constraint.s0, 3, 1),  constraint.cross_product));
                        terms.emplace_back(std::make_pair<int, double>(getUnknownIndex(constraint.s0, 2, 1), -constraint.cross_product));
                        
                        terms.emplace_back(std::make_pair<int, double>(getUnknownIndex(constraint.s1, 0, 0), 1.0));
                        terms.emplace_back(std::make_pair<int, double>(getUnknownIndex(constraint.s1, 1, 0), -1.0));
                        
                        system->addConstraint(terms, m_params.G1Weight);
                    }
                    {
                        vector< pair<int, double> > terms;
                        terms.reserve(6);
                        
                        terms.emplace_back(std::make_pair<int, double>(getUnknownIndex(constraint.s0, 3, 0), -constraint.cross_product));
                        terms.emplace_back(std::make_pair<int, double>(getUnknownIndex(constraint.s0, 2, 0),  constraint.cross_product));
                        terms.emplace_back(std::make_pair<int, double>(getUnknownIndex(constraint.s0, 3, 1), -constraint.dot_product));
                        terms.emplace_back(std::make_pair<int, double>(getUnknownIndex(constraint.s0, 2, 1),  constraint.dot_product));
                        
                        terms.emplace_back(std::make_pair<int, double>(getUnknownIndex(constraint.s1, 0, 1), 1.0));
                        terms.emplace_back(std::make_pair<int, double>(getUnknownIndex(constraint.s1, 1, 1), -1.0));
                        
                        system->addConstraint(terms, m_params.G1Weight);
                    }
                    
                }
#endif
                
                system->finalizeProblem();
                
                m_systems[i] = std::move(system);
            });
            
        }
    }
}

#ifdef WIN_ENV
__pragma(warning(pop))
#endif
