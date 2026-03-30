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

#include "composition/Composition"

#include "PuppetApplyVector.h"
#include "PuppetEffectVector.h"

namespace extensions
{
    namespace puppet
    {
        class ConcretePuppetApplyVector : public PuppetApplyVector
        {
            // A weak reference to the node that we are 'applying'
            std::weak_ptr<composition::GenericNode> _node;
            
            // Stores the output from the effect
            std::shared_ptr<const graphics::Group> _group;

        public:
            ConcretePuppetApplyVector(composition::GenericNode& node) :
                _node(node.genericNodeRef())
            {
            }
            
            void prepare() override
            {
                std::shared_ptr<composition::GenericNode> node = _node.lock();
                if (!node)
                    return;
                
                auto effect = node->effect();
                if (!(effect && effect->is<PuppetEffectVector>()))
                    return;
                auto& puppetEffect = effect->as<PuppetEffectVector>();
                
                // Retrieve the output from the effect
                _group = puppetEffect.outputVectors();
            }
            
            void apply() override
            {
                std::shared_ptr<composition::GenericNode> node = _node.lock();
                if (!node)
                    return;
                
                // Retrieve the root group from the src node
                auto& destGroup = node->contents().as<composition::VectorNodeContents>().mutableRootGroup();
                
                // Replacing the Paths is much more efficient than replacing the entire FigureGraphic
                // instance when undoing and redoing. Replacing the entire FigureGraphic with a new instance
                // requires sketchlib to track the removal and insertion of the Graphic instances. When
                // replacing the Path, sketchlib just swaps pointers to the Figure::Impl objects, which
                // is a lot faster.
                // Replacing the Path requires that the output group and the srce node have the same number
                // of children.
                if (destGroup.numChildren() == _group->numChildren())
                {
                    // Iterate thru FigureGraphics and update their curves
                    auto destIter = destGroup.begin();
                    auto groupIter = _group->begin();
                    
                    while (destIter != destGroup.end())
                    {
                        if (destIter->is<graphics::FigureGraphic>()
                            && groupIter->is<graphics::FigureGraphic>())
                        {
                            auto& destFigure = destIter->as<graphics::FigureGraphic>();
                            auto& groupFigure = groupIter->as<graphics::FigureGraphic>();
                            
                            shapes::Path& path = destFigure.mutableFigure().mutablePath();
                            path = groupFigure.figure().path();
                        }
                        
                        destIter++;
                        groupIter++;
                    }
                }
                else
                {
                    // Clear the children from the destGroup
                    destGroup.removeChildren(destGroup.begin(), destGroup.end());
                    
                    // Duplicate the output to the src node
                    graphics::GraphicDuplicator::duplicateChildren(*_group, destGroup);
                }
                
                // Clear out the _group member variable before the effect goes away
                _group = nullptr;
                
                // Remove the effect
                node->setEffect(nullptr);
            }
        };
    
        std::unique_ptr<PuppetApplyVector> PuppetApplyVector::create(composition::GenericNode& node)
        {
            // If puppet effect is not applied return
            auto effect = node.effect();
            if (!(effect && effect->is<PuppetEffectVector>()))
                return nullptr;
            
            return std::make_unique<ConcretePuppetApplyVector>(node);
        }
    }
}
