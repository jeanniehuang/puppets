//
//  ADOBE CONFIDENTIAL
//  __________________
//
//  Copyright 2021 Adobe
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

#ifndef EXTENSIONS_PUPPET_PUPPETIMAGE
#define EXTENSIONS_PUPPET_PUPPETIMAGE

#include "composition/effects/EffectFullRasterSource.h"
#include "composition/effects/EffectRasterImageProvider.h"

namespace extensions
{
    namespace puppet
    {
        /** An image that renders a source by applying a puppet mesh.
         */
        class PuppetImageProvider : public composition::EffectRasterImageProvider
        {
            mutable std::mutex _mutex;
            
            // the source being filtered
            std::unique_ptr<composition::EffectFullRasterSource> _source;
            
            // mesh and the size of the image coordinate system for the mesh
            std::unique_ptr<gpu::VertexBuffer> _vertices;
            std::unique_ptr<gpu::IndexBuffer> _indices;
            geom::Vector2f _size;
            
            // True indicates the puppet interprets the alpha as inverted for purpose of blending
            bool _isAlphaInverted;
            
            // whether to show the mesh overlay
            bool _showMesh = false;

        public:
            using super = composition::EffectRasterImageProvider;
            
            /** Create a puppet image.
                The source provides the image that is liquified.
                w, h define the logical size of the puppet image.
             */
            PuppetImageProvider(std::unique_ptr<composition::EffectFullRasterSource> source,
                                float w, float h, bool treatAlphaAsInverted = false);
            
            virtual ~PuppetImageProvider();
            
            /** Get the source.
             */
            composition::EffectFullRasterSource& source()
            {
                return *_source;
            }
            
            void setWarp(std::unique_ptr<gpu::VertexBuffer> vertices,
                         std::unique_ptr<gpu::IndexBuffer> indices,
                         const geom::Vector2f& size,
                         bool showMesh);
            
            /** Draw to destination. Note that the transform is from the bounds of the image
                to destination coords.
             */
            void draw(renderer::GPUTarget& target, const geom::Projectivity2f& transform,
                      const renderer::GPUCompositingOptions& options) const override;

            /** Returns the supported alpha format for the given requested format.
                PuppetImage's overridden draw method always draws premultiplied alpha. */
            renderer::AlphaFormat supportedAlphaFormat(renderer::AlphaFormat requestedFormat) const override;

            /** When true, the puppet image will be inverted, drawn, then inverted
                back. While nearly identical, this allows hardware blending to
                occur on the inverted image, which is desirable in cases such
                as editing masks.
             */
            bool treatAlphaAsInverted() const { return _isAlphaInverted; }
        };
    
    }
}

#endif /* EXTENSIONS_PUPPET_PUPPETIMAGE */
