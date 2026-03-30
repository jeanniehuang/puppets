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

#include <cctype>

#include "composition/Composition"
#include "renderer/image/Image"

#include "PuppetApplyRaster.h"
#include "PuppetEffectRaster.h"
#include "PuppetPrograms.h"

#define ENABLE_FRAME_CAPTURE_SCOPE 0

namespace extensions
{
    namespace puppet
    {
        /* For now, we use a default tile set that draws every source
           tile to the destination tile. */
        class PuppetSourceTileSet : public renderer::VirtualTextureArrayTraversal::SourceTileSet
        {
        public:
            bool unchanged() const override
            {
                return false;
            }

            bool isSource(uint32_t i, uint32_t j) const override
            {
                return true;
            }
        };

        /* For now, we use a default tile set array that draws
           every source tile to every destination tile. */
        class PuppetSourceTileSetArray : public renderer::VirtualTextureArrayTraversal::SourceTileSetArray
        {
            PuppetSourceTileSet _sourceTileSet;
        public:
            const renderer::VirtualTextureArrayTraversal::SourceTileSet& sourceTileSet(uint32_t i, uint32_t j) const override
            {
                return _sourceTileSet;
            }
        };

        /* A drawer that draws the provided puppet effect from a source tile to a destination tile. */
        class PuppetDrawer : public renderer::VirtualTextureArrayTraversal::Drawer
        {
            std::unique_ptr<gpu::VertexBuffer> _vertices;
            std::unique_ptr<gpu::IndexBuffer> _indices;
            bool _treatAlphaAsInverted;

            /** Sets up the appropriate blending mode for the given plane type.
                This method configures the GPU state for drawing different types of plane data.
             */
            void setBlendingFromPlaneType(renderer::GPUTarget& target,
                                          renderer::VirtualTextureArrayTraversal::PlaneType planeType)
            {
                switch (planeType)
                {
                case renderer::VirtualTextureArrayTraversal::PlaneType::Color:
                case renderer::VirtualTextureArrayTraversal::PlaneType::Mask:
                    // Use normal hardware blending of source on top of destination.
                    target.renderContext().enableBlending();
                    target.renderContext().setBlendMode(gpu::RenderContext::kOne, gpu::RenderContext::kOneMinusSrcAlpha);
                    break;
                case renderer::VirtualTextureArrayTraversal::PlaneType::Auxiliary:
                    // Use additive blending to sum all the values
                    target.renderContext().enableBlending();
                    target.renderContext().setBlendMode(gpu::RenderContext::kOne, gpu::RenderContext::kOne);
                    break;
                }
            }

        public:
            PuppetDrawer(PuppetEffectRaster& effect) :
                _treatAlphaAsInverted(effect.treatAlphaAsInverted())
            {
                // Obtain the triangles from the effect using the consolidated method
                // If vertex count causes uint16_t overflow, indices will be nullptr and vertices will be duplicated per triangle
                effect.createWarpBuffers(_vertices, _indices);
            }

            void drawTileToTile(renderer::GPUTarget& target,
                                const gpu::Texture& srcTexture,
                                gpu::Texture& dstTexture,
                                const geom::Affinity2f& imageToDstTileNDC,
                                const geom::Affinity2f& imageToSrcTileTextureCoords,
                                const geom::Affinity2f& imageToImageTextureCoords,
                                renderer::VirtualTextureArrayTraversal::PlaneType planeType,
                                bool clear) override
            {
                // Prepare the GPU context
                setBlendingFromPlaneType(target, planeType);
                target.renderContext().bindColorBuffer(dstTexture);

                // Can't use hardware filtering because source is unmultiplied alpha
                const gpu::Sampler sourceSampler = target.sampler_nn(srcTexture);

                // Draw the source texture to the destination
                renderer::PuppetPreviewProgramDrawer drawer;
                drawer.draw(target, imageToDstTileNDC, imageToSrcTileTextureCoords, 
                            sourceSampler, *_vertices, _indices.get(), _treatAlphaAsInverted, clear, /* showMesh = */ false);
            }
        };

        class ConcretePuppetApplyRaster : public PuppetApplyRaster
        {
            // A weak reference to the node that we are 'applying'
            std::weak_ptr<composition::DrawingLayerNode> _node;
            // The output of the apply operation, containing the new surface for the
            // content and potentially the mask.
            renderer::VirtualTextureArrayTraversal::SurfaceResult _preparedResult;

        public:
            ConcretePuppetApplyRaster(composition::DrawingLayerNode& node) :
                _node(node.drawingLayerNodeRef())
            {
            }
            
            void prepare() override
            {
                std::shared_ptr<composition::DrawingLayerNode> node = _node.lock();
                if (!node)
                    return;
                
                auto effect = node->effect();
                if (!(effect && effect->is<PuppetEffectRaster>()))
                    return;
                auto& puppetEffect = effect->as<PuppetEffectRaster>();

                // Make sure the effect is updated
                base::JobGroup group;
                puppetEffect.update(group);
                group.wait();
                
                renderer::GPUTargetFactory::LocalTarget target(renderer::GPUTargetFactory::instance());
                
                gpu::AutoSaveState saveState((*target).renderContext());
                saveState.saveColorBuffer();
                saveState.saveScissor();
                (*target).renderContext().disableScissor();

                PuppetSourceTileSetArray sourceTileSetArray;
                PuppetDrawer drawer(puppetEffect);

#if ENABLE_FRAME_CAPTURE_SCOPE
                (*target).renderContext().flush();
                (*target).renderContext().debugBeginCaptureScope("Puppet apply");
#endif

                // We use the full raster source due to the multiple-puppet case. In such
                // a situation, we need to preserve a meaningful z-order for the puppets, which
                // cannot be done by incrementally drawing source tiles to a single destination
                // tile. Imagine the simplest case, where a destination tile has two source tiles
                // each containing their own puppet. The z-order should be specified by which triangles
                // occur first in the vertex / index buffer, but if we draw the source tiles incrementally,
                // in this case, it will be based on the arbitrary order in which we visit the source
                // tiles. For example, if the second puppet (on the second source tile) is actually
                // underneath the first puppet (on the first source tile) according to the triangles,
                // then we'll have a problem. The second puppet will be drawn after the first puppet
                // because it is visited later, meaning it will incorrectly be drawn on top.
                renderer::VirtualTextureArrayTraversal::AccumulateDrawnTilesOptions options;
                options.skipEmptySourceTiles = true;
                options.useFullRasterSource = true;


                // We only pass mask options if the effect is applied to masked content
                std::optional<renderer::VirtualTextureArrayTraversal::AccumulateDrawnTilesOptions> maskOptions = std::nullopt;
                if (puppetEffect.target() == composition::Effect::Target::maskedContent)
                {
                    maskOptions = renderer::VirtualTextureArrayTraversal::AccumulateDrawnTilesOptions();
                    maskOptions.value().skipEmptySourceTiles = true;
                    maskOptions.value().useFullRasterSource = true;
                }

                _preparedResult = renderer::VirtualTextureArrayTraversal::accumulateDrawnTiles(*target,
                                                                                               *node,
                                                                                               drawer,
                                                                                               sourceTileSetArray,
                                                                                               options,
                                                                                               maskOptions);

#if ENABLE_FRAME_CAPTURE_SCOPE
                (*target).renderContext().flush();
                (*target).renderContext().debugEndCaptureScope("Puppet apply");
#endif
            }
            
            void apply() override
            {
                std::shared_ptr<composition::DrawingLayerNode> node = _node.lock();
                if (!node)
                    return;
                
                // Invalidate the effect to ensure content outside the layer bounds are 'cleaned up'
                // by the renderer. While this should theoretically be done via the drawing layer
                // rather than through the effect being removed, the drawing layer clips damage to its
                // bounds. Thus, the most straightforward means of ensuring that the area outside the
                // bounds are redrawn is to fully invalidate the effect first.
                auto effect = node->effect();
                if (effect && effect->is<extensions::puppet::PuppetEffect>())
                {
                    auto& puppetEffect = effect->as<extensions::puppet::PuppetEffect>();
                    puppetEffect.invalidate();
                }
                
                // Remove the effect
                node->setEffect(nullptr);
                
                renderer::GPUTargetFactory::LocalTarget target(renderer::GPUTargetFactory::instance());

                // Set the contents
                if (_preparedResult.contentSurface)
                {

                    const geom::Box2f damage(0, _preparedResult.contentSurface->width(),
                                             0, _preparedResult.contentSurface->height());
                    node->layer().replaceSurface(damage, std::move(_preparedResult.contentSurface));
                }
                
                // Set the mask
                if (_preparedResult.maskSurface && node->mask())
                {
                    composition::DrawingLayerNode* mask = node->mask();
                    const geom::Box2f damage(0, _preparedResult.maskSurface->width(),
                                             0, _preparedResult.maskSurface->height());
                    mask->layer().replaceSurface(damage, std::move(_preparedResult.maskSurface));
                }
            }
        };
    
        std::unique_ptr<PuppetApplyRaster> PuppetApplyRaster::create(composition::DrawingLayerNode& node)
        {
            // If puppet effect is not applied return
            auto effect = node.effect();
            if (!(effect && effect->is<PuppetEffectRaster>()))
                return nullptr;
            
            return std::make_unique<ConcretePuppetApplyRaster>(node);
        }
    }
}
