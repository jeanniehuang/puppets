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

#include "../PuppetTypes.h"

#include "puppet_warp/puppet_accelerator.h"

#include "bezier_segment_base.h"
#include "util.h"

/** \file
 * A bezier segment is a single cubic bezier element embedded on a mesh.
 */

namespace extensions
{
    namespace puppet
    {
        // given a 2D point, find the corresponding mesh location.
        MeshLocation findMeshCoordinate(const geom::Vector2f &pt,
                                        optimtools::PuppetMeshGeometry<double> &accelerator)
        {
            MeshLocation l;
            //Eigen::Matrix<Point, Eigen::Dynamic, 1> ptEigen;
            Eigen::Array<double, 2, 1> ptEigen;
            ptEigen[0] = pt.x;
            ptEigen[1] = pt.y;
            accelerator.nearest_triangle_barycentric(l.tri, l.puppet_index, l.uvw(0), l.uvw(1), l.uvw(2), ptEigen);
            return l;
        }

        struct BezierSample : public BezierSampleBase
        {
            // the location of this sample on the mesh
            MeshLocation l;
        };

        template <typename T, typename R>
        R Vec2FContainer<T, R>::x() const
        {
            return point2d(0);
        }

        template <typename T, typename R>
        R Vec2FContainer<T, R>::y() const
        {
            return point2d(1);
        }

        template <typename T, typename R>
        R& Vec2FContainer<T, R>::x()
        {
            return point2d(0);
        }

        template <typename T, typename R>
        R& Vec2FContainer<T, R>::y()
        {
            return point2d(1);
        }

        // a cubmic bezier primitive embedded on a mesh
        struct BezierSegment : public BezierSegmentBase<BezierSample, Vec2FContainer<geom::Vector2f, float>>
        {
            using VecContainer = Vec2FContainer<geom::Vector2f, float>;
            
            // compute the approriate triangle and UVW coordinates for each sample
            /*void bindSamplesToMesh(const mesh_traits<double, 2>::PointVector& points, const mesh_traits<double, 2>::TriangleVector& triangles)
             {
             for (auto &s : samples)
             {
             s.l = findMeshCoordinate(evaluate(s.t), points, triangles);
             }
             }*/
            void bindSamplesToMesh(optimtools::PuppetMeshGeometry<double> &accelerator)
            {
                for (auto &s : samples)
                {
                    s.l = findMeshCoordinate(evaluate(s.t), accelerator);
                }
            }
            
            // subdivides the bezier segment. This is currently only used for performance testing and is not correct subdivision.
            void subdivide(BezierSegment &s0, BezierSegment &s1) const
            {
                s0.p[0] = evaluate(0.0f / 6.0f);
                s0.p[1] = evaluate(1.0f / 6.0f);
                s0.p[2] = evaluate(2.0f / 6.0f);
                s0.p[3] = evaluate(3.0f / 6.0f);
                
                s1.p[0] = evaluate(3.0f / 6.0f);
                s1.p[1] = evaluate(4.0f / 6.0f);
                s1.p[2] = evaluate(5.0f / 6.0f);
                s1.p[3] = evaluate(6.0f / 6.0f);
            }
        };
    }
}
