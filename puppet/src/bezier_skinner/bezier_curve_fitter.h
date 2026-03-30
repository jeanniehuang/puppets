/*************************************************************************
* ADOBE CONFIDENTIAL
* ___________________
*
*  Copyright 2025 Adobe
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

#include <vector>

#include "geom/Vector.h"

namespace extensions
{
    namespace puppet
    {
        class BezierRep
        {
        public:
            static std::vector<geom::Vector2f>            FitCurve   (const std::vector<geom::Vector2f> &d, double error);

            /*
                *  Reparameterize:
                *  Given set of points and their parameterization, try to find
                *   a better parameterization.
                *
                */
            static std::vector<double>                    Reparameterize(const geom::Vector2f d[], size_t first, size_t last,
                                                                         const std::vector<double> &u, const geom::Vector2f bezCurve[4]);
            /*
                *  ChordLengthParameterize :
                *  Assign parameter values to digitized points
                *  using relative distances between points.
                */
            static std::vector<double>                    ChordLengthParameterize(const geom::Vector2f d[], size_t first, size_t last);
            /*
                *  ComputeMaxError :
                *  Find the maximum squared distance of digitized points
                *  to fitted curve.
            */
            static double                                 ComputeMaxError(const geom::Vector2f d[], size_t first, size_t last, const geom::Vector2f bezCurve[4],
                                                                          const std::vector<double> &u, size_t *splitPoint2D=NULL);
            static double                                 ComputeAvgError(const geom::Vector2f d[], size_t first, size_t last, const geom::Vector2f bezCurve[4],
                                                                          const std::vector<double> &u);
            
        };
    }
}
