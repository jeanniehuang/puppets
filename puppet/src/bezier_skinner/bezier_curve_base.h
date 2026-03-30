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

#include "bezier_segment.h"

namespace extensions
{
    namespace puppet
    {
        struct C0Constraint
        {
            C0Constraint() { s0 = -1; s1 = -1; }
            C0Constraint(int _s0, int _s1) { s0 = _s0; s1 = _s1; }
            // indices of the segments involved.
            int s0{-1}, s1{-1};
            
            // constraint is satisfied if
            // segments[s0].p3 == segments[s1].p0
        };

        struct C1Constraint
        {
            C1Constraint()
            {
                s0 = -1;
                s1 = -1;
            }
            
            C1Constraint(int _s0, int _s1) { s0 = _s0; s1 = _s1; }
            // indices of the segments involved.
            int s0{-1}, s1{-1};
            
            // constraint is satisfied if
            // segments[s0].p3 - segments[s0].p2 == segments[s1].p1 - segments[s1].p0
        };

        struct G1Constraint
        {
            G1Constraint()
            {
                s0 = -1;
                s1 = -1;
                s0_p = 2;
                s1_p = 1;
                len0 = 1;
                len1 = 1;
            }
            
            G1Constraint(int _s0, const BezierSegment& seg0, int _s1, const BezierSegment& seg1, double G1LengthThreshold = 1e-2)
            {
                s0 = _s0;
                s1 = _s1;
                s0_p = 2;
                s1_p = 1;
                
                auto length =  [] (typename BezierSegment::VecContainer::type p) {
                    
                    BezierSegment::VecContainer point{}; point.point2d = p;
                    
                    return sqrt(point.x() * point.x() + point.y() * point.y());
                };
                
                len0 = length(seg0.p[3] - seg0.p[2]);
                if(len0  <= G1LengthThreshold)
                {
                    len0 = length(seg0.p[3] - seg0.p[1]);
                    s0_p = 1;
                }
                if(len0  <= G1LengthThreshold)
                {
                    len0 = length(seg0.p[3] - seg0.p[0]);
                    s0_p = 0;
                }
                
                len1 = length(seg1.p[1] - seg1.p[0]);
                if(len1  <= G1LengthThreshold)
                {
                    len1 = length(seg1.p[2] - seg1.p[0]);
                    s1_p = 2;
                }
                if(len1 <= G1LengthThreshold)
                {
                    len0 = length(seg1.p[3] - seg1.p[0]);
                    s1_p = 3;
                }
            }
            // indices of the segments involved.
            int s0{}, s1{};
            int s0_p{}, s1_p{};
            float len0{}, len1{};
            
            // constraint is satisfied if
            // dot((segments[s0].p3 - segments[s0].p2),(segments[s1].p1 - segments[s1].p0)) == len(segments[s0].p3 - segments[s0].p2) * len(segments[s1].p1 - segments[s1].p0)
        };

        // a list of bezier segments with explicitly encoded continuity
        struct BezierCurveBase
        {
            std::vector<BezierSegment> segments;
            std::vector<C0Constraint> C0Constraints;
            std::vector<C1Constraint> C1Constraints;
            std::vector<G1Constraint> G1Constraints;
            
            BezierCurveBase() = default;
            
            // computes the error across each constraint
            float constraintError() const
            {
                float sum = 0.0f;
                for (auto &c : C0Constraints) sum += constraintError(c);
                for (auto &c : C1Constraints) sum += constraintError(c);
                for (auto &c : G1Constraints) sum += constraintError(c);
                return sum;
            }
            
            float constraintError(const C0Constraint &c) const
            {
                const BezierSegment &s0 = segments[c.s0];
                const BezierSegment &s1 = segments[c.s1];
                
                BezierSegment::VecContainer v0; v0.point2d = s0.p[3];
                BezierSegment::VecContainer v1; v1.point2d = s1.p[0];
                
                return ((v0.x() - v1.x()) * (v0.x() - v1.x()) + (v0.y() - v1.y()) * (v0.y() - v1.y()));
            }
            
            float constraintError(const C1Constraint &c, double C1LengthThreshold = 1e-2) const
            {
                const BezierSegment& seg0 = segments[c.s0];
                const BezierSegment& seg1 = segments[c.s1];
                
                BezierSegment::VecContainer s0_vec; s0_vec.point2d = seg0.p[3] - seg0.p[2];
                BezierSegment::VecContainer s1_vec; s1_vec.point2d = seg1.p[1] - seg1.p[0];
                
                bool normalize = false;
                
                auto s0_vec_length = sqrt((s0_vec.x() * s0_vec.x()) + (s0_vec.y() * s0_vec.y()));
                if (s0_vec_length <= C1LengthThreshold)
                {
                    s0_vec.point2d = seg0.p[3] - seg0.p[1];
                    normalize = true;
                }
                
                s0_vec_length = sqrt((s0_vec.x() * s0_vec.x()) + (s0_vec.y() * s0_vec.y()));
                if (s0_vec_length <= C1LengthThreshold)
                {
                    s0_vec.point2d = seg0.p[3] - seg0.p[0];
                    normalize = true;
                }
                
                s0_vec_length = sqrt((s0_vec.x() * s0_vec.x()) + (s0_vec.y() * s0_vec.y()));
                if (normalize && s0_vec_length >= C1LengthThreshold)
                {
                    double val = (double)1.0 / s0_vec_length;
                    s0_vec.x() *= static_cast<float>(val);
                    s0_vec.y() *= static_cast<float>(val);
                }
                
                auto s1_vec_length = sqrt((s1_vec.x() * s1_vec.x()) + (s1_vec.y() * s1_vec.y()));
                if (s1_vec_length <= C1LengthThreshold)
                {
                    s1_vec.point2d = seg1.p[2] - seg1.p[0];
                    normalize = true;
                }
                
                s1_vec_length = sqrt((s1_vec.x() * s1_vec.x()) + (s1_vec.y() * s1_vec.y()));
                if (s1_vec_length <= C1LengthThreshold)
                {
                    s1_vec.point2d = seg1.p[3] - seg1.p[0];
                    normalize = true;
                }
                
                s1_vec_length = sqrt((s1_vec.x() * s1_vec.x()) + (s1_vec.y() * s1_vec.y()));
                if (normalize && s1_vec_length >= C1LengthThreshold)
                {
                    double val = (double)1.0 / s1_vec_length;
                    s1_vec.x() *= static_cast<float>(val);
                    s1_vec.y() *= static_cast<float>(val);
                }
                
                const auto& v0 = s0_vec; const auto& v1 = s1_vec;
                return ((v0.x() - v1.x()) * (v0.x() - v1.x()) + (v0.y() - v1.y()) * (v0.y() - v1.y()));
            }
            
            float constraintError(const G1Constraint &c) const
            {
                const BezierSegment &s0 = segments[c.s0];
                const BezierSegment &s1 = segments[c.s1];
                
                BezierSegment::VecContainer v0; v0.point2d = s0.p[3] - s0.p[c.s0_p];
                BezierSegment::VecContainer v1; v1.point2d = s1.p[c.s1_p] - s1.p[0];
                
                auto dotP = (v0.x() * v1.x()) + (v0.y() * v1.y());
                
                return std::abs(c.len0 * c.len1 - dotP);
            }
        };
    }
}
