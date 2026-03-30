/*************************************************************************
* ADOBE CONFIDENTIAL
* ___________________
*
*  Copyright 2022 Adobe
*  All Rights Reserved.
*
* NOTICE:  All information contained herein is, and remains
* the property of Adobe and its suppliers, if any. The intellectual
* and technical concepts contained herein are proprietary to Adobe
* and its suppliers and are protected by all applicable intellectual
* property laws, including trade secret and copyright laws.
* Dissemination of this information or reproduction of this material
* is strictly forbidden unless prior written permission is obtained
* from Adobe.
**************************************************************************/

#pragma once

/** \file
 * A bezier segment is a single cubic bezier element embedded on a mesh.
 */

#include "optimtools/all.h"

#include "geom/Vector.h"

namespace extensions
{
    namespace puppet
    {
        struct BezierSampleBase
        {
            // the position along the bezier curve
            float t{};
        };

        namespace
        {
            //Hacker's Delight, 2nd Edition, Chapter 11
            int icbrt(unsigned x) {
                int s;
                unsigned y, b, y2;
                
                y2 = 0;
                y = 0;
                for (s = 30; s >= 0; s = s - 3) {
                    y2 = 4*y2;
                    y = 2*y;
                    b = (3*(y2 + y) + 1) << s;
                    if (x >= b) {
                        x = x - b;
                        y2 = y2 + 2*y + 1;
                        y = y + 1;
                    }
                }
                return y;
            }
        }

        // T = 2d point type, which supports addition and scalar multiplication
        template <typename T = geom::Vector2f, typename R = float>
        struct Vec2FContainer
        {
            using type = T;

            T point2d;
            
            // To be implemented at client side
            R x() const;

            // To be implemented at client side
            R y() const;

            // To be implemented at client side
            R& x();

            // To be implemented at client side
            R& y();
        };

        // a cubmic bezier primitive embedded on a mesh
        // T = BezierSampleBase, V = Vec2FContainer<T, R>
        template <typename T = BezierSampleBase, typename V = Vec2FContainer<geom::Vector2f, float>>
        struct BezierSegmentBase
        {
            // uniformly sample the curve at a given estimated distance between samples.
            void initSamples(float sampleStride)
            {
                int length = static_cast<int>(estimateLength());
                const int sampleCount = std::min(std::max(8, static_cast<int>(length * icbrt(length) / sampleStride)),50);
                samples.resize(sampleCount);
                for (int i = 0; i < sampleCount; i++)
                {
                    T &s = samples[i];
                    s.t = float(i) / (sampleCount - 1.0f);
                }
            }

            typename V::type evaluate(float t) const
            {
                float tm = 1.0f - t;
                return  p[0] * (tm * tm * tm) +
                        p[1] * (3.0f * tm * tm * t)  +
                        p[2] * (3.0f * tm * t  * t)  +
                        p[3] * (t  * t  * t);
            }

            float estimateLength(int parts = 8) const
            {
                float sum = 0.0f;
                for (int i = 0; i < parts; i++)
                {
                    auto evalV0 = evaluate(i / (float)parts);
                    auto evalV1 = evaluate((i + 1) / (float)parts);
                    
                    V v0 {}; v0.point2d = evalV0;
                    V v1 {}; v1.point2d = evalV1;
                    
                    auto res = sqrt((v0.x() - v1.x()) * (v0.x() - v1.x()) + (v0.y() - v1.y()) * (v0.y() - v1.y()));

                    sum += res;
                }
                return sum;
            }

            // the four control points
            typename V::type p[4];

            // the samples used for curve fitting.
            std::vector<T> samples;
        };
    }
}
