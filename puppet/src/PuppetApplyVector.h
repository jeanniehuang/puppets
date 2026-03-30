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

#ifndef EXTENSIONS_PUPPET_PUPPETAPPLYVECTOR
#define EXTENSIONS_PUPPET_PUPPETAPPLYVECTOR

#include <memory>

namespace composition
{
    class GenericNode;
}

namespace extensions
{
    namespace puppet
    {
        /** Applies a puppet effect to a vector layer.
         */
        class PuppetApplyVector
        {
        public:
            /** Apply the effect that is attached to the layer and remove the
                effect. The effect is applied only to the layer, not its mask.
             */
            static std::unique_ptr<PuppetApplyVector> create(composition::GenericNode& layer);

            virtual ~PuppetApplyVector() = default;

            /** Prepares for applying the puppet effect. May be run on a background
                thread so long as nothing is concurrently modifying the model.
             */
            virtual void prepare() = 0;
            
            /** Applies the result removing the effect. Must run on the main thread.
             */
            virtual void apply() = 0;
        };
    }
}

#endif /* EXTENSIONS_PUPPET_PUPPETAPPLYVECTOR */
