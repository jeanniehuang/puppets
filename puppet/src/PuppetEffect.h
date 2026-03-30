/*************************************************************************
 *
 * ADOBE CONFIDENTIAL
 * ___________________
 *
 *  Copyright 2021 Adobe Systems Incorporated
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

#ifndef EXTENSIONS_PUPPET_PUPPETEFFECT
#define EXTENSIONS_PUPPET_PUPPETEFFECT

#include <vector>

#include "composition/effects/Effect.h"
#include "composition/Element.h"
#include "geom/Vector.h"

#include "PuppetTypes.h"
#include "stroker/PuppetStroker.h"

#include "../api/model/PuppetSettings.h" // TY_TODO: figure out non relative path. TASK: https://jira.corp.adobe.com/browse/GEMINI-30948

namespace base
{
    class JobDispatcher;
}

namespace composition
{
    class EffectFullRasterSource;
}

namespace extensions
{
    namespace puppet
    {
        struct PuppetStrokeSample;

        /** Base class for raster and vector puppet effects
         */
        class PuppetEffect : public composition::Effect,
                             public composition::Element
        {
        protected:
            class StateData
            {
            public:
                // Wrapper around the stroker state that has ownership over the puppets
                std::unique_ptr<PuppetStroker::PuppetStrokerState> puppetStrokerState;

                StateData() = default;
                
                // Delete copy constructor and copy assignment to prevent accidental copying
                StateData(const StateData&) = delete;
                StateData& operator=(const StateData&) = delete;
            };
            
            struct Memento : public composition::ElementMemento
            {
                std::shared_ptr<StateData> _data;

                void combine(ElementMemento& after) override
                {
                }
            };
            
            class MementoProvider : public composition::ElementMementoProvider
            {
                PuppetEffect& _effect;
            public:
                MementoProvider(PuppetEffect& effect) :
                    _effect(effect)
                {
                }
                
                std::unique_ptr<composition::ElementMemento>
                getState(composition::ElementMemento* m) const
                {
                    if (m)
                        return nullptr;
                    return _effect.getState();
                }
            };
            
            // the last saved/restored state unless contents have been modified since
            mutable std::shared_ptr<StateData> _currentState;
            
            // Settings applied to the puppets
            model::PuppetSettings _puppetSettings;

            // Stroker for managing warping operations on a background thread
            std::unique_ptr<PuppetStroker> _puppetStroker;

        public:
            class Tests;
            friend class PuppetTestHelpers;

            virtual ~PuppetEffect() = default;
            
            /** Constructs the initial mesh.
                This could be a long-running operation, and should be avoided on the main thread.
                Clients should wait for this method to return before calling any other PuppetEffect methods.
                It checks progress and may throw a Cancel exception if the operation is cancelled.
             */
            virtual void initialize(const model::PuppetSettings& puppetSettings,
                                    const base::Progress& progress) = 0;
            
            /** Releases resources, including puppets, used by the effect in order to reclaim memory.
                After calling this method, clients will need to call initialize() again before calling any
                other method in this class.  */
            virtual void deInitialize() = 0;
            
            /** Prepare for editing the puppet at the provided sample. A hit radius sets a threshold for
                detecting hits slightly outside the mesh.
                @return true if a puppet was hit and dragging will be valid, false otherwise
             */
            virtual bool beginSamples(const PuppetStrokeSample& sample, Scalar hitRadius=0);

            /** Sculpt the selected puppet by dragging the current handle to the provided sample
             */
            virtual void addSample(const PuppetStrokeSample& sample);

            /** End the drag operation for the puppet
             */
            virtual void endSamples();

            /** Set whether to show the mesh overlay
             */
            void setShowMesh(bool showMesh);
            
            /** Enable or disable automatic pinning */
            void setAutoPinEnabled(bool enabled);
            
            /** Adjust the fraction of vertices automatically pinned (furthest X%) */
            void setAutoPinFraction(double fraction);

            /** Restore to the state in the memento and set the memento
                state to the state prior to the restore.
             */
             void swapState(composition::ElementMemento& m) override;

            /** Access for the puppet (const only).
             */
            const PuppetT& selectedPuppet() const;
            
            /** Returns whether there is currently content of the effect that is projected 
                outside the traditional bounds of the node. For raster effects, this 
                corresponds to having any vertex (that was originally within the layer bounds) 
                now fall outside the layer bounds. For vector effects, there is no concept 
                of destination bounds, so this method always returns false.
             */
            virtual bool hasContentOutsideDstBounds() const = 0;
            
            /** Invalidate the effect by dispatching a change with infinite bounds.
                This will result in a full redraw of the node at the renderer level
             */
            void invalidate();

        protected:
            PuppetEffect(composition::ElementCollection* collection,
                         std::shared_ptr<volatile base::JobDispatcher> jobDispatcher) :
                composition::Element(collection),
                _puppetStroker(std::make_unique<PuppetStroker>(jobDispatcher))
            {
            }

            /** Initialize the puppets based on the source image passed in.
                Checks progress and may throw a Cancel exception if the operation is cancelled.
             */
            void initializePuppets(composition::EffectFullRasterSource& image,
                                   const geom::Vector2f& imageOrigin,
                                   bool treatAlphaAsInverted,
                                   const base::Progress& progress);
            
            /** Release the memory used by the puppet meshes for this effect. */
            void deInitializePuppets();

            /** Directly sets the provided puppet as the only puppet for the effect.
                Should only be used by Tests.
             */
            virtual void setPuppet(std::shared_ptr<PuppetT> puppet);

        private:
            bool selectPuppetAtPoint(const geom::Vector2f& point);
            
            std::unique_ptr<composition::ElementMemento> getState();
            void saveState();
            
            void dispatchChanging(const geom::Box2f& damage, bool recorded);
        };
    }
}

#endif
