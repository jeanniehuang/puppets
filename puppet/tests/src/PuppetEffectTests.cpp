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
//  and its suppliers are protected by all applicable intellectual
//  property laws, including trade secret and copyright laws.
//  Dissemination of this information or reproduction of this material
//  is strictly forbidden unless prior written permission is obtained
//  from Adobe.
//

#include "CPTestCase.h"

#include "composition/Composition"

#include "sketch-foundation/base/Time.h"
#include "sketch-foundation/numerics/StandardFunctions.h"

#include "api/control/PuppetFunctions.h"
#include "PuppetEffect.h"
#include "tests/PuppetTestHelpers.h"

namespace extensions
{
    namespace puppet
    {
        using namespace optimtools;
        USING_OPTIMTOOLS_MESH_TYPES(typename,double,2);
        
        using PuppetTestHelpers = extensions::puppet::PuppetTestHelpers;
        
        class PuppetEffect::Tests
        {
            void testPerformanceInner(model::LayerRef targetLayer, int numIterations)
            {
                // Draw black circle (produces more triangles than square)
                geom::Vector4f black(0,0,0,1);
                PuppetTestHelpers::drawCircle(targetLayer, 500, 500, 300, black);
                
                // Add puppet effect to layer
                auto puppetSettings = model::PuppetSettings();
                auto defaultProgress = base::Progress();
                auto puppetEffect = control::PuppetFunctions::createEffect(targetLayer, puppetSettings, defaultProgress);
                control::PuppetFunctions::initializeEffect(puppetEffect, puppetSettings, defaultProgress);
                control::PuppetFunctions::attachEffect(targetLayer, puppetEffect);
                
                // Get puppet and effect references
                auto& effect = targetLayer->node().effect()->as<extensions::puppet::PuppetEffect>();
                const extensions::puppet::PuppetT &puppet = effect.selectedPuppet();
                
                // Create initial handle at center
                PuppetStrokeSample initialPos;
                initialPos.position = geom::Vector2f(500, 500);
                initialPos.time = 0.0;
                effect.beginSamples(initialPos);

                // Run many iterations with deterministic but varying drag positions
                for (int i = 0; i < numIterations; ++i) {
                    // Generate deterministic drag point using spiral pattern
                    float angle = (i * 17.0f) * 3.14159f / 180.0f; // 17 degrees per iteration
                    float spiralRadius = 50.0f + (i * 2.0f); // Expanding spiral
                    
                    // Clamp spiral radius to stay within bounds
                    spiralRadius = numerics::clamp(10.0f, 400.0f, spiralRadius);
                    
                    // Calculate new drag position
                    float dragX = 500.0f + spiralRadius * cos(angle);
                    float dragY = 500.0f + spiralRadius * sin(angle);
                    
                    // Clamp to image bounds
                    dragX = numerics::clamp(50.0f, 950.0f, dragX);
                    dragY = numerics::clamp(50.0f, 950.0f, dragY);

                    PuppetStrokeSample dragPos;
                    dragPos.position = geom::Vector2f(dragX, dragY);
                    dragPos.time = (i+1) * 0.016f; // Simulate ~60Hz input

                    // Move the handle to new position
                    effect.addSample(dragPos);
                }
                
                // Finish the puppet manipulation
                effect.endSamples();
                
                CPAssertTrue(effect._puppetStroker->puppets().size() > 0);

                // Apply the effect to the layer
                control::PuppetFunctions::applyEffect(targetLayer, true /*removeFromJournal*/);
                
                // Validate that passing in true for 'removeFromJournal' also cleared the puppets
                CPAssertTrue(effect._puppetStroker->puppets().size() == 0);
            }
            
            void testUndoRedoInner(model::LayerRef targetLayer, model::Drawing& drawing)
            {
                // Draw black square to create puppet geometry
                geom::Vector4f black(0,0,0,1);
                PuppetTestHelpers::drawSquare(targetLayer, 100, 100, 400, 400, black);
                
                // Add puppet effect to layer
                model::PuppetSettings puppetSettings;
                auto defaultProgress = base::Progress();
                auto puppetEffect = control::PuppetFunctions::createEffect(targetLayer, puppetSettings, defaultProgress);
                control::PuppetFunctions::initializeEffect(puppetEffect, puppetSettings, defaultProgress);
                control::PuppetFunctions::attachEffect(targetLayer, puppetEffect);

                // Get puppet and effect references
                auto& effect = targetLayer->node().effect()->as<extensions::puppet::PuppetEffect>();
                const extensions::puppet::PuppetT& puppet = effect.selectedPuppet();

                // Step 1: Create a puppet - already done above
                
                // Step 2: Modify the puppet with the begin / add / end sample method
                drawing.beginUndoGroup();
                PuppetStrokeSample firstDragSample;
                firstDragSample.position = geom::Vector2f(250.0f, 250.0f); // center of square
                firstDragSample.time = 0.0;
                effect.beginSamples(firstDragSample);
                firstDragSample.position += geom::Vector2f(50.0f, 30.0f);     // drag amount
                firstDragSample.time += 0.4;
                effect.addSample(firstDragSample);
                
                // call update() to cause the triangle depth order to be calculated
                base::JobGroup group;
                effect.update(group);
                group.wait();
                
                effect.endSamples();
                drawing.endUndoGroup();
                
                // Step 3: Copy the base mesh geometry and triangle depth order after first modification
                auto firstModifiedState = puppet.copyState();
                // Step 4: Modify the puppet with the sample methods again
                drawing.beginUndoGroup();
                PuppetStrokeSample secondDragSample;
                secondDragSample.position = geom::Vector2f(200.0f, 200.0f); // different point
                secondDragSample.time = 0.8;
                effect.beginSamples(secondDragSample);
                secondDragSample.position += geom::Vector2f(30.0f, -40.0f); // different drag
                secondDragSample.time += 0.4;
                effect.addSample(secondDragSample);
                
                // call update() to cause the triangle depth order to be calculated
                effect.update(group);
                group.wait();
                
                effect.endSamples();
                drawing.endUndoGroup();
                
                // Step 5: Copy the base mesh geometry and triangle depth order after second modification
                auto secondModifiedState = puppet.copyState();
                
                // Step 6: Undo (should restore to first modified state)
                model::Drawing::RollBack(drawing).step();
                
                // Step 7: Verify we have the same geometry and triangle depth order from step 3
                puppet.compareState(*firstModifiedState);
                
                // Step 8: Redo (should restore to second modified state)
                model::Drawing::RollForward(drawing).step();
                
                // Step 9: Verify we have the same geometry and triangle depth order from step 5
                puppet.compareState(*secondModifiedState);
                
                CPAssertTrue(effect._puppetStroker->puppets().size() > 0);
                
                // Apply the effect to the layer
                control::PuppetFunctions::applyEffect(targetLayer, true /*removeFromJournal*/);
                
                // Validate that passing in true for 'removeFromJournal' also cleared the puppets
                CPAssertTrue(effect._puppetStroker->puppets().size() == 0);
            }
            
        public:
            void testPuppetPerformance()
            {
                const uint32_t w = 1000;
                const uint32_t h = 1000;

                // Increase this number to test more performance cycles
                const int numIterations = 1;

                // Create a performance timer to measure the total time
                base::PerformanceTimer timer("Puppet performance iterate warper");

                // Make a drawing
                std::unique_ptr<model::Drawing> drawing = model::Drawing::createForTest();
                
                // Add raster layer to root
                model::LayerRef targetLayer = PuppetTestHelpers::addRasterLayer(*drawing, w, h);
                
                // Run performance code
                testPerformanceInner(targetLayer, numIterations);
                
                // Add vector layer to root
                targetLayer = PuppetTestHelpers::addVectorLayer(*drawing);
                
                // Run performance code
                testPerformanceInner(targetLayer, numIterations);
            }
            
            /** Verify that undo and redo update the puppet's base mesh geometry.
             */
            void testUndoRedo()
            {
                const uint32_t w = 500;
                const uint32_t h = 500;
                
                // Make a drawing
                std::unique_ptr<model::Drawing> drawing = model::Drawing::createForTest();
                
                // Add raster layer to root
                model::LayerRef targetLayer = PuppetTestHelpers::addRasterLayer(*drawing, w, h);
                
                // Run undo/redo code
                testUndoRedoInner(targetLayer, *drawing);

                // Add vector layer to root
                targetLayer = PuppetTestHelpers::addVectorLayer(*drawing);
                
                // Run undo/redo code
                testUndoRedoInner(targetLayer, *drawing);
            }
        };
    }
}

CPSUITE(PuppetEffectTests, extensions::puppet::PuppetEffect::Tests)
CPTEST(testPuppetPerformance)
CPTEST(testUndoRedo)
CPENDSUITE
