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

#include "renderer/surface/Surface"

#include "PuppetImage.h"
#include "PuppetPrograms.h"

namespace extensions
{
    namespace puppet
    {

        PuppetImageProvider::PuppetImageProvider(std::unique_ptr<composition::EffectFullRasterSource> source,
                                                 float w, float h, bool treatAlphaAsInverted) :
            super(w, h),
            _source(std::move(source)),
            _isAlphaInverted(treatAlphaAsInverted),
            _showMesh(false)
        {
        }
        
        PuppetImageProvider::~PuppetImageProvider()
        {
        }

        void PuppetImageProvider::setWarp(std::unique_ptr<gpu::VertexBuffer> vertices,
                                          std::unique_ptr<gpu::IndexBuffer> indices,
                                          const geom::Vector2f& size,
                                          bool showMesh)
        {
            std::lock_guard<std::mutex> lock(_mutex);
            
            _vertices = std::move(vertices);
            _indices = std::move(indices);
            _size = size;
            _showMesh = showMesh;
        }

        void PuppetImageProvider::draw(renderer::GPUTarget& target,
                                       const geom::Projectivity2f& transform,
                                       const renderer::GPUCompositingOptions& options) const
        {
            // NOTE: could probably use the blend compositor here. That would avoid
            // needing another and fully support all blend options.

            std::lock_guard<std::mutex> lock(_mutex);

            if (!_vertices)
                return;

            // Get the source data with resolution closest to the destination. This
            // will give the best quality result.
            std::shared_ptr<renderer::ConstVirtualTexture> source = _source->texture(target, transform);
            renderer::VirtualTexture::Read sourceTexture(target, *source);

            // can't use hardware filtering because source is un-mul
            const gpu::Sampler sourceSampler = target.sampler_nn(*sourceTexture);

            // Calculate coordinate transformation matrices
            const geom::Affinity2f vertexToDst = transform.toAffinity() * geom::Affinity2f::scale(width()/_size.x, height()/_size.y);
            const geom::Affinity2f vertexToSrc = geom::Affinity2f::scale(1.0/_size.x, 1.0/_size.y);

            // Use hardware over-blending such that overlapping puppets
            // are blended together.
            target.renderContext().enableBlending();
            target.renderContext().setBlendMode(gpu::RenderContext::kOne, gpu::RenderContext::kOneMinusSrcAlpha);

            // Draw the source to the destination
            renderer::PuppetPreviewProgramDrawer drawer;
            drawer.draw(target, vertexToDst, vertexToSrc, sourceSampler, *_vertices, _indices.get(), _isAlphaInverted, /* clear = */ true, _showMesh);
        }

        renderer::AlphaFormat PuppetImageProvider::supportedAlphaFormat(renderer::AlphaFormat requestedFormat) const
        {
            // PuppetImage always returns premultiplied alpha regardless of what's requested
            return renderer::AlphaFormat::premul;
        }

    }
}
