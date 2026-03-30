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

#include "PuppetEditor.h"
#include "PuppetEffect.h"
#include "stroker/PuppetStrokeSample.h"

namespace extensions
{
    namespace puppet
    {
        using namespace optimtools;

        PuppetEditor::PuppetEditor(std::shared_ptr<PuppetEffect> effect) :
            _effect(std::move(effect))
        {
        }

        bool PuppetEditor::beginSamples(const PuppetStrokeSample& sample,
                                        Scalar hitRadius)
        {
            // Pass through directly to effect and return result
            return _effect->beginSamples(sample, hitRadius);
        }
    
        void PuppetEditor::addSample(const PuppetStrokeSample& sample)
        {
            // Pass through directly to effect
            _effect->addSample(sample);
        }
    
        void PuppetEditor::endSamples()
        {
            // Pass through to effect
            _effect->endSamples();
        }
    }
}
