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

#ifndef EXTENSIONS_PUPPET_PUPPETEDITOR
#define EXTENSIONS_PUPPET_PUPPETEDITOR

#include "geom/Geom"
#include "PuppetTypes.h"

namespace extensions
{
    namespace puppet
    {
        class PuppetEffect;
        struct PuppetStrokeSample;

        /**
         * Implements an editing session for a PuppetEffect where samples are added to deform the puppet
         */
        class PuppetEditor
        {
            /** The puppet effect that this editor manipulates */
            std::shared_ptr<PuppetEffect> _effect;
            
        public:
            /**
             * Constructs a PuppetEditor for the given effect. */
            PuppetEditor(std::shared_ptr<PuppetEffect> effect);
            
            /** Begins a puppet manipulation at the specified sample. Setting a hit radius allows hits to be
             *  detected even if they lie slightly outside the mesh.
             *  @return true if a puppet was hit and dragging will be valid, false otherwise.
             */
            bool beginSamples(const PuppetStrokeSample& sample, Scalar hitRadius=0);

            /** Continues the puppet manipulation with a new sample. */
            void addSample(const PuppetStrokeSample& sample);

            /** Ends the current puppet manipulation. */
            void endSamples();
        };
    }
}

#endif /* EXTENSIONS_PUPPET_PUPPETEDITOR */
