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

#include "base/Base"

#include "composition/Composition"
#include "composition/effects/EffectFullRasterSource.h"
#include "renderer/gpu/GPU"
#include "renderer/image/Image"

#include "sketchlib/control/StrokeSample.h"
#include "sketchlib/model/RasterLayerContents.h"
#include "sketchlib/model/VectorLayerContents.h"

#include "PuppetApplyRaster.h"
#include "PuppetApplyVector.h"
#include "PuppetEditor.h"
#include "PuppetEffectRaster.h"
#include "PuppetEffectVector.h"
#include "PuppetFunctions.h"
#include "stroker/PuppetStrokeSample.h"

#define LOGGING 0

#if LOGGING
#define LOGMSG(msg) std::cout << msg << std::endl
#else
#define LOGMSG(msg)
#endif
namespace control
{
    namespace
    {
        class DisableRecording
        {
            composition::Journal& _journal;

        public:
            DisableRecording(composition::Journal& journal) :
                _journal(journal)
            {
                _journal.disableRecording();
            }
            ~DisableRecording()
            {
                _journal.enableRecording();
            }
        };

        std::shared_ptr<extensions::puppet::PuppetEffect> getPuppetEffect(const model::LayerRef& layer)
        {
            using namespace extensions::puppet;
            
            composition::Node& node = layer->node();
            
            // if puppet effect is not applied return
            auto effect = node.effect();
            if (!(effect && effect->is<PuppetEffect>()))
                return nullptr;
            
            return std::static_pointer_cast<PuppetEffect>(effect);
        }
    
        std::shared_ptr<composition::Effect> addRasterEffect(model::RasterLayerContentsRef layerContents,
                                                             composition::Effect::Target target,
                                                             bool treatAlphaAsInverted,
                                                             const model::PuppetSettings& puppetSettings)
        {
            composition::DrawingLayerNode& node = layerContents->node();

            // get layer size
            const float w = node.width();
            const float h = node.height();
            
            composition::Composition& composition = layerContents->owner();
            std::shared_ptr<renderer::VirtualTextureArrayAllocator> allocator = composition.factories().allocator();

            // create the source for the surrogate from the layer contents
            composition::EffectFullRasterSource::Params params;
            params.bounds(geom::Box2f(0, w, 0, h));
            params.allocator(std::move(allocator));
            params.target(target);
            params.alphaFormat(renderer::AlphaFormat::unmul);
            params.areaThreshold(1);

            std::unique_ptr<composition::EffectFullRasterSource> source = composition::EffectFullRasterSource::create(node, params);
            composition.flushOutput().check();
            
            // make the puppet effect
            auto puppet = extensions::puppet::PuppetEffectRaster::create(&composition.journal().elements(),
                                                                         composition::Universe::instance().generalQueue(),
                                                                         std::move(source),
                                                                         w, h, treatAlphaAsInverted);
            return puppet;
        }
    
        std::shared_ptr<composition::Effect> addVectorEffect(model::VectorLayerContentsRef layerContents,
                                                             composition::Node& layerNode,
                                                             const model::PuppetSettings& puppetSettings)
        {
            // make the puppet effect
            composition::Composition& composition = layerContents->owner();
            auto puppet = extensions::puppet::PuppetEffectVector::create(&composition.journal().elements(),
                                                                         composition::Universe::instance().generalQueue(),
                                                                         layerNode.as<composition::GenericNode>(),
                                                                         layerContents->owner());
            return puppet;
        }
    }
    
    class ConcretePuppetEditor : public PuppetEditor
    {
        extensions::puppet::PuppetEditor _editor;
        
    public:
        ConcretePuppetEditor(std::shared_ptr<extensions::puppet::PuppetEffect> effect) :
            _editor(std::move(effect))
        {
        }
        
        bool beginSamples(const StrokeSample& sample,
                          const util::AffineTransform& transform,
                          double hitRadius) override
        {
            extensions::puppet::PuppetStrokeSample s{convert(sample, transform), sample.time};
            return _editor.beginSamples(s, hitRadius);
        }

        void appendSample(const StrokeSample& sample,
                          const util::AffineTransform& transform) override
        {
            // Forward both position and time to the editor
            extensions::puppet::PuppetStrokeSample s{convert(sample, transform), sample.time};
            _editor.addSample(s);
        }
        
        void endSamples() override
        {
            _editor.endSamples();
        }
        
    private:
        geom::Vector2f convert(const StrokeSample& sample,
                               const util::AffineTransform& transform)
        {
            // transform sample point
            const util::Point pt = transform.transform(util::Point(sample.x, sample.y));
            return geom::Vector2f(pt.x, pt.y);
        }
    };


    std::shared_ptr<composition::Effect> PuppetFunctions::createEffect(const model::LayerRef& layer,
                                                                       const model::PuppetSettings& puppetSettings,
                                                                       const base::Progress& progress)
    {
        LOGMSG("PuppetFunctions::createEffect");
        // Works only with unmultiplied colors
        ASK_CHECK_NO_PII(base::FeatureSet::instance().isFeatureAvailable("unmultipliedColors"));

        using namespace extensions::puppet;
        
        if (progress.isCancelled())
            return nullptr;
        
        if (!layer->contents()->is<model::RasterLayerContents>() && !layer->contents()->is<model::VectorLayerContents>())
            return nullptr;
        
        if (layer->node().effect())
        {
            // first remove the effect
            layer->node().setEffect(nullptr);
        }
        
        // Now create the effect and return it.
        std::shared_ptr<composition::Effect> puppet;
        {
            if (layer->contents()->is<model::RasterLayerContents>())
            {
                // Always transform just the content. We can revisit mask support in the future
                composition::Effect::Target target = composition::Effect::Target::content;

                // Masks have two modes - reveal and hide - which affect what values are treated as the
                // 'content' of the mask for bounding box and transformation purposes. In hide mode,
                // 255 is treated as the opaque content while 0 is treated as the background, fitting in
                // to our existing conception of how puppets work. However, in reveal mode, 0 is treated
                // as the opaque content while 255 is treated as the background. This requires special
                // handling, such as ensuring the puppet triangulation is calculated based on pixels with
                // alpha [0, 254] rather than [1, 255]. Additionally, the blending of overlapping puppets
                // must be adjusted. As such, we pass the following parameter into the puppet machinery
                // to ensure that the alpha is handled in accordance to the mask mode.
                bool treatAlphaAsInverted = layer->parentRelation() == model::ParentRelation::masked &&
                                            layer->maskMode() == model::MaskMode::reveal;

                // create the source for the surrogate from the layer contents
                puppet = addRasterEffect(layer->contents()->as<model::RasterLayerContents>(),
                                         target,
                                         treatAlphaAsInverted,
                                         puppetSettings);
            }
            else if (layer->contents()->is<model::VectorLayerContents>())
            {
                puppet = addVectorEffect(layer->contents()->as<model::VectorLayerContents>(),
                                         layer->node(),
                                         puppetSettings);
            }
            else
            {
                ASK_REQUIRE_NO_PII(false, "Layer type mismatch");
            }
            
            return puppet;
        }
    }
    
    void PuppetFunctions::initializeEffect(std::shared_ptr<composition::Effect>& effect,
                                           const model::PuppetSettings& puppetSettings,
                                           const base::Progress& progress)
    {
        LOGMSG("PuppetFunctions::initializeEffect\n");

        // Return early, if its not a puppet effect
        if (!(effect && effect->is<extensions::puppet::PuppetEffect>()))
            return;
        
        extensions::puppet::PuppetEffect& puppetEffect = effect->as<extensions::puppet::PuppetEffect>();
        puppetEffect.initialize(puppetSettings, progress);
    }

    void PuppetFunctions::attachEffect(const model::LayerRef& layer,
                                       std::shared_ptr<composition::Effect> effect)
    {
        LOGMSG("PuppetFunctions::attachEffect\n");
        // Works only with unmultiplied colors
        ASK_CHECK_NO_PII(base::FeatureSet::instance().isFeatureAvailable("unmultipliedColors"));
        
        using namespace extensions::puppet;
        
        // Return early if its not a puppet effect
        if (!(effect && effect->is<extensions::puppet::PuppetEffect>()))
            return;

        // Return early for non-raster and non-vector layers
        if (!layer->contents()->is<model::RasterLayerContents>() && !layer->contents()->is<model::VectorLayerContents>())
            return;
        
        if (layer->node().effect())
        {
            // first remove the effect
            layer->node().setEffect(nullptr);
        }
        
        // set effect on node
        layer->node().setEffect(effect);
    }

    void PuppetFunctions::removeEffect(const model::LayerRef& layer)
    {
        LOGMSG("PuppetFunctions::removeEffect");
        using namespace extensions::puppet;
        
        composition::Node& node = layer->node();
        
        // if puppet effect is not applied return
        auto effect = node.effect();
        if (!(effect && effect->is<PuppetEffect>()))
            return;
        
        node.setEffect(nullptr);
    }
    
    void PuppetFunctions::applyEffect(const model::LayerRef& layer, bool removeFromJournal)
    {
        LOGMSG("PuppetFunctions::applyEffect");
#if LOG_SCULPTING_PERFORMANCE
        base::PerformanceTimer timer("Sculpting: apply effect");
#endif
        
        using namespace extensions::puppet;
        
        // can only apply to raster or vector but don't leave effects lying around
        if (layer->contents()->type() != model::LayerContents::Type::raster
            && layer->contents()->type() != model::LayerContents::Type::vector)
        {
            removeEffect(layer);
            return;
        }
        
        std::shared_ptr<PuppetEffect> puppetEffect = getPuppetEffect(layer);
        if (!puppetEffect)
            return;
        
        if (layer->contents()->is<model::RasterLayerContents>())
        {
            auto contents = layer->contents()->as<model::RasterLayerContents>();
            composition::DrawingLayerNode& node = contents->node();
            
            // Apply the effect
            auto algorithm = PuppetApplyRaster::create(node);
            if (!algorithm)
                return;
            
            algorithm->prepare();
            algorithm->apply();
        }
        else if (layer->contents()->is<model::VectorLayerContents>())
        {
            auto& node = layer->node().as<composition::GenericNode>();
            
            // Apply the effect
            auto algorithm = PuppetApplyVector::create(node);
            if (!algorithm)
                return;
            
            algorithm->prepare();
            algorithm->apply();
        }
        
        if (removeFromJournal)
        {
            puppetEffect->deInitialize();
            
            composition::Journal& journal = layer->owner().journal();
            
            // Disable recording; will be re-enabled when this object goes out of scope
            DisableRecording disable(journal);
            
            // Remove the element from the journal. This will remove all mementos in the
            // journal that were associated with this effect.
            journal.elements().removeElement(*puppetEffect);
            
            // Re-add the element to the journal. This is needed because when the effect
            // is destructed, it tries again to remove itself from the journal, which will
            // cause the reference count of the journal to be out of sync.
            journal.elements().insertElement(*puppetEffect);
        }
    }
    
    bool PuppetFunctions::hasContentOutsideDstBounds(const model::ConstLayerRef& layer)
    {
        using namespace extensions::puppet;
        
        // Get the puppet effect from the node
        const composition::Node& node = layer->contents()->node();
        auto effect = node.effect();
        if (!(effect && effect->is<PuppetEffect>()))
        {
            // If we can't find a puppet effect, no content outside bounds
            return false;
        }
        
        const PuppetEffect& puppetEffect = effect->as<const PuppetEffect>();
        
        // Check if the puppet effect has content projected outside destination bounds
        return puppetEffect.hasContentOutsideDstBounds();
    }

    std::unique_ptr<PuppetEditor> PuppetFunctions::createEditor(const model::LayerRef& layer)
    {
        LOGMSG("PuppetFunctions::createEditor");
        using namespace extensions::puppet;

        model::LayerContentsRef contents = layer->contents();
        if ( !(contents->is<model::RasterLayerContents>()
               || contents->is<model::VectorLayerContents>()) )
            return nullptr;
        
        composition::Node& node = contents->node();
        
        auto effect = node.effect();
        if (!(effect && effect->is<PuppetEffect>()))
            return nullptr;
        
        std::shared_ptr<PuppetEffect> puppet = std::static_pointer_cast<PuppetEffect>(effect);
        return std::make_unique<ConcretePuppetEditor>(std::move(puppet));
    }
    
    void PuppetFunctions::setAutoPin(const model::LayerRef& layer, bool enabled) {
        if (!layer)
            return;

        auto puppetEffect = getPuppetEffect(layer);
        if (puppetEffect)
        {
            puppetEffect->setAutoPinEnabled(enabled);
        }
    }

    void PuppetFunctions::setAutoPinFraction(const model::LayerRef& layer, double fraction) {
        if (!layer)
            return;
        
        auto puppetEffect = getPuppetEffect(layer);
        if (puppetEffect)
        {
            puppetEffect->setAutoPinFraction(fraction);
        }
    }

    
}
