//
//  ADOBE CONFIDENTIAL
//  __________________
//
//  Copyright 2025 Adobe
//  All Rights Reserved.
//
//  NOTICE:  All information contained herein is, and remains
//  the property of Adobe and its suppliers, if any. The intellectual
//  and technical concepts contained herein are proprietary to Adobe
//  and its suppliers and are protected by all applicable intellectual
//  property laws, including trade secret and copyright laws.
//  Dissemination of this information or reproduction of this material
//  is strictly forbidden unless prior written permission is obtained
//  from Adobe.
//

#ifndef EXTENSIONS_PUPPET_PUPPETPSLGAUTOZONEADAPTER_H
#define EXTENSIONS_PUPPET_PUPPETPSLGAUTOZONEADAPTER_H

#include <vector>

#include "optimtools/all.h"
#include "optimtools_extras/mesh/all.h"
#include "composition/effects/EffectFullRasterSource.h"

namespace extensions
{
    namespace puppet
    {
        /**
         * Result structure containing a PSLG and its associated metrics
         */
        struct PSLGResult
        {
            optimtools::PSLG<double, int64_t> pslg;

            // Area estimate in pixels as returned by LayerColorFunctions::findZoneBoundariesWithArea.
            // However, this is not the actual area covered by the PSLG, since the simplification step
            // occurs after the area calculation.
            int32_t areaEstimate;
            
            // Perimeter length in pixels, calculated from the original presimplified figure boundaries.
            // This represents the total length returned by AutoZone, so it can be used in conjunction with
            // the area estimate.
            float perimeterEstimate;
            
            PSLGResult() = default;
            PSLGResult(optimtools::PSLG<double, int64_t> p, int32_t area, float perimEst) 
                : pslg(std::move(p)), areaEstimate(area), perimeterEstimate(perimEst) {}
        };
        
        /**
         * Adapter class for creating PSLGs using the AutoZone algorithm
         */
        class PuppetPSLGAutoZoneAdapter
        {
        public:
            /**
             * Create PSLGs from an image source using the AutoZone algorithm
             *
             * There are three algorithmic steps to accomplish this goal:
             *    1. We first dilate the provided source image by the outline tolerance. This closes gaps,
             *      but also ensures that our boundaries do not shrink too far into the shape when the
             *      paths are later simplified. The resulting image at this point is a single-channel binarized
             *      image, where 255 represents the content and 0 represents the background.
             *    2. Next, we extract the outlines from the dilated image using AutoZone. This efficiently
             *      produces a high-point-density collection of linear paths that encompass the dilated
             *      region. Each figure represents a single connected component. The connected
             *      component may be built up of several paths, since the connected components may
             *      have holes. Holes are have different path orientation than content - they are clockwise.
             *    3. Finally, we simplify each of the linear paths using the Douglas-Peucker algorithm given
             *      the provided tolerance. This ensures a smoother outline while also providing a more
             *      manageable number of points.
             *
             * @param source The raster source to extract PSLGs from
             * @param sourceLevel The level to extract from the raster source
             * @param outlineTolerance Tolerance for outline generation and dilation amount. A larger
             *                         number means more dilation and more simplification.
             * @param invertSource Whether the source image should be inverted before processing.
             * @return Vector of PSLGResults containing PSLGs and their area estimates
             */
            static std::vector<PSLGResult> createPSLGs(composition::EffectFullRasterSource& source,
                                                       int sourceLevel,
                                                       double outlineTolerance,
                                                       bool invertSource);

        };
    }
}

#endif /* EXTENSIONS_PUPPET_PUPPETPSLGAUTOZONEADAPTER_H */
