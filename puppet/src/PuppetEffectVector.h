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

#ifndef EXTENSIONS_PUPPET_PUPPETEFFECTVECTOR
#define EXTENSIONS_PUPPET_PUPPETEFFECTVECTOR

#include <memory>

#include "PuppetEffect.h"

namespace composition
{
    class Composition;
    class GenericNode;
}

namespace graphics
{
    class Group;
}

namespace extensions
{
    namespace puppet
    {
        class PuppetEffectVector : public PuppetEffect
        {
        public:
            static std::shared_ptr<PuppetEffectVector>
                create(composition::ElementCollection* collection,
                       std::shared_ptr<volatile base::JobDispatcher> jobDispatcher,
                       composition::GenericNode& vectorNode,
                       composition::Composition& composition);
            
            PuppetEffectVector(composition::ElementCollection* collection,
                               std::shared_ptr<volatile base::JobDispatcher> jobDispatcher) :
                PuppetEffect(collection, jobDispatcher)
            {
            }
            virtual ~PuppetEffectVector() = default;
            
            /** Retrieve the output vectors that were deformed by the puppet effect
             */
            virtual std::shared_ptr<const graphics::Group> outputVectors() const = 0;
        };
    }
}

#endif
