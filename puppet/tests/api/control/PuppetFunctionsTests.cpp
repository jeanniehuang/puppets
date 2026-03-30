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


#include "CPTestCase.h"

#include "composition/Composition"

#include "api/control/PuppetFunctions.h"
#include "PuppetEffectRaster.h"
#include "PuppetEffectVector.h"
#include "tests/PuppetTestHelpers.h"

namespace control
{
    using namespace optimtools;
    USING_OPTIMTOOLS_MESH_TYPES(typename,double,2);
    
    using extensions::puppet::PuppetTestHelpers;
    
    class PuppetFunctions::Tests
    {
    public:
        
        /** Verifies that adding and removing a Puppet effect manipulates the
         model as expected.
         */
        void testAddRemoveForRasterLayer()
        {
            // Make a drawing
            std::unique_ptr<model::Drawing> drawing = model::Drawing::createForTest();
            
            // Add raster layer to root
            model::LayerRef rasterLayer = PuppetTestHelpers::addRasterLayer(*drawing);
            
            // Draw black square
            PuppetTestHelpers::drawSquare(rasterLayer, 0, 0, 2, 2, geom::Vector4f(0,0,0,1));
            
            // Add Puppet effect
            auto puppetSettings = model::PuppetSettings();
            auto defaultProgress = base::Progress();
            auto puppetEffect = PuppetFunctions::createEffect(rasterLayer, puppetSettings, defaultProgress);
            PuppetFunctions::initializeEffect(puppetEffect, puppetSettings, defaultProgress);
            PuppetFunctions::attachEffect(rasterLayer, puppetEffect);
            
            
            // Check the effect was applied
            auto& rasterNode = rasterLayer->node().as<composition::DrawingLayerNode>();
            CPAssertTrue(rasterNode.effect() != nullptr);
            CPAssertTrue(rasterNode.effect()->is<extensions::puppet::PuppetEffectRaster>());
            
            // Remove
            removeEffect(rasterLayer);
            
            // Check gone
            CPAssertTrue(rasterNode.effect() == nullptr);
        }
        
        void testAddRemoveForVectorLayer()
        {
            // Make a drawing
            std::unique_ptr<model::Drawing> drawing = model::Drawing::createForTest();

            // Add vector layer to root
            model::LayerRef vectorLayer = PuppetTestHelpers::addVectorLayer(*drawing);

            // Draw black square
            PuppetTestHelpers::drawSquare(vectorLayer, 2, 2, 8, 8, geom::Vector4f(0,0,0,1));

            // Add Puppet effect
            auto puppetSettings = model::PuppetSettings();
            auto defaultProgress = base::Progress();
            auto puppetEffect = PuppetFunctions::createEffect(vectorLayer, puppetSettings, defaultProgress);
            PuppetFunctions::initializeEffect(puppetEffect, puppetSettings, defaultProgress);
            PuppetFunctions::attachEffect(vectorLayer, puppetEffect);

            // Check the effect was applied
            auto& vectorNode = vectorLayer->node().as<composition::GenericNode>();
            CPAssertTrue(vectorNode.effect() != nullptr);
            CPAssertTrue(vectorNode.effect()->is<extensions::puppet::PuppetEffectVector>());

            // Remove
            removeEffect(vectorLayer);

            // Check gone
            CPAssertTrue(vectorNode.effect() == nullptr);
        }

        void testDrawRasterLayer()
        {
            const bool result = PuppetTestHelpers::testDrawRasterLayerWithOptions(false);
            CPAssertTrue(result);
        }

        void testApplyRasterLayer()
        {
            // Parameters for test
            const uint32_t w = 9;           // size of layer
            const uint32_t h = 4;
            const uint32_t tilesize = 4;    // size of layer tiles
            const float horizontalShiftPixels = 1.0f;
            const bool targetMask = false;

            // Create the input (a 3x2 black square in the top left)
            std::unique_ptr<model::Drawing> drawing;
            model::LayerRef targetLayer = PuppetTestHelpers::createTiledDrawing(drawing, targetMask, w, h, tilesize);
            geom::Vector4f black(0,0,0,1);
            PuppetTestHelpers::drawSquare(targetLayer, 0, 0, 3, 2, black);

            // Add puppet effect
            auto puppetSettings = model::PuppetSettings();
            auto defaultProgress = base::Progress();
            auto puppetEffect = PuppetFunctions::createEffect(targetLayer, puppetSettings, defaultProgress);
            PuppetFunctions::initializeEffect(puppetEffect, puppetSettings, defaultProgress);
            PuppetFunctions::attachEffect(targetLayer, puppetEffect);

            // Create and configure the puppet
            auto shiftPuppet = extensions::puppet::PuppetTestHelpers::createRectanglePuppet(3, 2, 3);
            extensions::puppet::PuppetTestHelpers::applyHorizontalShiftToWarper(*shiftPuppet, 0, 3, 0, 2, horizontalShiftPixels);

            CPAssertTrue(shiftPuppet != nullptr);
            CPAssertTrue(PuppetTestHelpers::applyPuppet(targetLayer, shiftPuppet));
            
            // Get the results
            std::vector<uint32_t> result = PuppetTestHelpers::getRasterLayerPixels(targetLayer);

            // Create the expected results (a 3x2 black square in the top left, shifted over one pixel)
            model::LayerRef resultLayer = PuppetTestHelpers::createTiledDrawing(drawing, targetMask, w, h, tilesize);
            PuppetTestHelpers::drawSquare(resultLayer, 1, 0, 4, 2, black);
            std::vector<uint32_t> expectedResult = PuppetTestHelpers::getRasterLayerPixels(resultLayer);

            CPAssertTrue(PuppetTestHelpers::equalPixels(expectedResult, result, 1));
        }
        
        void testApplyVectorLayer()
        {
            // Parameters for test
            const uint32_t tilesize = 4;    // size of layer tiles
            const float horizontalShiftPixels = 1.0f;
            
            // Make a drawing with tile size
            std::unique_ptr<model::Drawing> drawing = model::Drawing::createForTest(tilesize);
            
            // Add vector layer to root with puppet effect
            model::LayerRef targetLayer = PuppetTestHelpers::addVectorLayer(*drawing);
            
            // Create the input (a 3x2 black square in the top left)
            geom::Vector4f black(0,0,0,1);
            PuppetTestHelpers::drawSquare(targetLayer, 0, 0, 3, 2, black);

            // Add puppet effect
            auto puppetSettings = model::PuppetSettings();
            auto defaultProgress = base::Progress();
            auto puppetEffect = PuppetFunctions::createEffect(targetLayer, puppetSettings, defaultProgress);
            PuppetFunctions::initializeEffect(puppetEffect, puppetSettings, defaultProgress);
            PuppetFunctions::attachEffect(targetLayer, puppetEffect);

            // Create and configure the puppet
            auto shiftPuppet = extensions::puppet::PuppetTestHelpers::createRectanglePuppet(3, 2, 3);
            PuppetTestHelpers::applyHorizontalShiftToWarper(*shiftPuppet, 0, 3, 0, 2, horizontalShiftPixels);

            CPAssertTrue(shiftPuppet != nullptr);
            CPAssertTrue(PuppetTestHelpers::applyPuppet(targetLayer, shiftPuppet));
            
            // Get the results
            const graphics::Group& group = PuppetTestHelpers::getVectorLayerRootGroup(targetLayer);
            
            // Validate that the points match expected results
            const auto& figureGraphic = group.getFirstChild().as<graphics::FigureGraphic>();
            const auto& path = figureGraphic.figure().path();
            const auto& curve = path.curves[0];
            
            constexpr float kEpsilon = 0.001;
            CPAssertTrue(curve.degree == 3);  // curve always bumped up to degree 3 in puppet code
            CPAssertTrue(curve.pts.size() == 13);  // closed square should have 13 points in degree 3
            CPAssertTrue(std::abs(curve.pts[0].x - 1) < kEpsilon && std::abs(curve.pts[0].y - 0) < kEpsilon);
            CPAssertTrue(std::abs(curve.pts[3].x - 4) < kEpsilon && std::abs(curve.pts[3].y - 0) < kEpsilon);
            CPAssertTrue(std::abs(curve.pts[6].x - 4) < kEpsilon && std::abs(curve.pts[6].y - 2) < kEpsilon);
            CPAssertTrue(std::abs(curve.pts[9].x - 1) < kEpsilon && std::abs(curve.pts[9].y - 2) < kEpsilon);
        }

    };
}

CPSUITE(PuppetFunctionsTests, control::PuppetFunctions::Tests)
CPTEST(testAddRemoveForRasterLayer)
CPTEST(testAddRemoveForVectorLayer)
CPTEST(testDrawRasterLayer)
CPTEST(testApplyRasterLayer)
CPTEST(testApplyVectorLayer)
CPENDSUITE
