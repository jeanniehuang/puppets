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

#ifndef EXTENSIONS_PUPPET_PUPPETEFFECTRASTER
#define EXTENSIONS_PUPPET_PUPPETEFFECTRASTER

#include "PuppetEffect.h"

namespace extensions
{
    namespace puppet
    {
        /** PuppetEffect subclass that is designed to apply the deformed puppets to raster layers
         */
        class PuppetEffectRaster : public PuppetEffect
        {
        protected:
            PuppetEffectRaster(composition::ElementCollection* collection,
                               std::shared_ptr<volatile base::JobDispatcher> jobDispatcher) :
                PuppetEffect(collection, jobDispatcher)
            {
            }

        public:
            static std::shared_ptr<PuppetEffectRaster>
            create(composition::ElementCollection* collection,
                   std::shared_ptr<volatile base::JobDispatcher> jobDispatcher,
                   std::unique_ptr<composition::EffectFullRasterSource> source,
                   float w, float h, bool treatAlphaAsInverted = false);
            
            virtual ~PuppetEffectRaster() = default;
            
            /** Get the vertex and index buffers required to draw the current puppet.
                When the vertex count would cause uint16_t index overflow, the index buffer
                will be set to nullptr and vertices will be duplicated per triangle. */
            virtual void createWarpBuffers(std::unique_ptr<gpu::VertexBuffer>& outVertices,
                                           std::unique_ptr<gpu::IndexBuffer>& outIndices) const = 0;

            /** Get whether the alpha channel of the source is treated as inverted for the
                purposes of generating the triangulation and blending multiple puppets. For
                example, when calculating the triangulation, pixels with the value 255 will
                be treated as the background. Note that the output of this effect will have
                the same alpha behavior as the input, so if the input is 'inverted', the output
                will still be inverted (i.e 255 will not be flipped to 0). */
            virtual bool treatAlphaAsInverted() const = 0;
        };
    }
}

#endif
