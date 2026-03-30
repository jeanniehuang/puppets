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

#ifndef CONTROL_PUPPETFUNCTIONS_H
#define CONTROL_PUPPETFUNCTIONS_H

#include "sketchlib/model/Layer.h"
#include "../model/PuppetSettings.h"

namespace base
{
    class Progress;
}

namespace composition
{
    class Effect;
}

namespace control
{

    class StrokeSample;

    class PuppetEditor
    {
    public:
        virtual ~PuppetEditor() = default;
        
        /** Begin interacting with the puppet. The transform maps the sample
            to the puppet coordinates. Hit radius can be supplied to allow a hit to
            be detected, even if it lies slightly outside the mesh.
            @return true if a puppet was hit and dragging will be valid, false otherwise
         */
        virtual bool beginSamples(const StrokeSample& sample,
                                  const util::AffineTransform& transform,
                                  double hitRadius=0) = 0;

        /** Continue interacting with the puppet. The transform maps the sample
            to the puppet coordinates.
         */
        virtual void appendSample(const StrokeSample& sample,
                                  const util::AffineTransform& transform) = 0;
        
        virtual void endSamples() = 0;
    };

    class PuppetFunctions
    {
    public:
        class Tests;
        
        /** Creates a puppet effect for the layer. If an effect is already attached to the layer, its removed before the new effect is created.
            The new effect that is returned is un-initialized and not attached to the layer,
            & needs to be initialized and attached to the node corresponding to the layer for it to take effect.
            Checks progress and may throw a Cancel exception if the operation is cancelled. */
        static std::shared_ptr<composition::Effect> createEffect(const model::LayerRef& layer,
                                                                 const model::PuppetSettings& puppetSettings,
                                                                 const base::Progress& progress);

        /** Initialization may be time-consuming, so clients are allowed to call this on a background thread.
            No other modifications to the layer or effect should be done until this method returns.
            Checks progress and may throw a Cancel exception if the operation is cancelled. */
        static void initializeEffect(std::shared_ptr<composition::Effect>& effect,
                                     const model::PuppetSettings& puppetSettings,
                                     const base::Progress& progress);

        /** Adds the effect to the layer. Needs to be called on the main thread.
            Assumes that the effect was created for the given layer using 'createEffect()'
            and was initialized using 'initializeEffect() */
        static void attachEffect(const model::LayerRef& layer,
                                 std::shared_ptr<composition::Effect> effect);
        
        /** Removes the puppet effect attached to the layer. */
        static void removeEffect(const model::LayerRef& layer);
        
        /** Applies the puppet effect attached by attachEffect() to the layer contents.
            The parameters specified when adding the effect are used.
            The effect is removed.
            If 'removeFromJournal' is true, all mementos associated with the effect are removed from
            the journal and the effect is de-initialized.
         */
        static void applyEffect(const model::LayerRef& layer, bool removeFromJournal);
        
        /** Returns whether there is currently content of the puppet effect on the given 
            layer that is projected outside the traditional bounds of the node. For raster 
            layers, this corresponds to having any vertex (that was originally within the 
            layer bounds) now fall outside the layer bounds. For vector layers, there is 
            no concept of destination bounds, so this always returns false.
         */
        static bool hasContentOutsideDstBounds(const model::ConstLayerRef& layer);

        static std::unique_ptr<PuppetEditor> createEditor(const model::LayerRef& layer);
        
        /** Enable or disable automatic pinning for the active puppet effect (if any) */
        static void setAutoPin(const model::LayerRef& layer, bool enabled);
        
        /** Update the fraction of vertices (furthest from drag handle) to auto-pin */
        static void setAutoPinFraction(const model::LayerRef& layer, double fraction);
    };

}

#endif /* CONTROL_PUPPETFUNCTIONS_H */
