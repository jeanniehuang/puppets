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

#ifndef TESTS_PUPPETTESTHELPERS
#define TESTS_PUPPETTESTHELPERS

#include <memory>
#include <cstdint>
#include <vector>
#include <string>

#include "sketchlib/control/DrawingInitialization.h"
#include "sketchlib/control/StrokeSample.h"
#include "sketchlib/model/Drawing.h"
#include "sketchlib/model/ImageDataProvider.h"
#include "sketchlib/model/LayerGroupContents.h"
#include "sketchlib/model/MaskAreaSnapshot.h"
#include "sketchlib/model/RasterLayerContents.h"
#include "sketchlib/model/VectorLayerContents.h"

#include "optimtools/all.h"
#include "SculptingPuppet.h"
#include "PuppetTypes.h"

namespace extensions
{
    namespace puppet
    {
        class PuppetTestHelpers
        {
        public:
            // Puppet creation and manipulation
            /** Creates a trivial puppet with a rectangle mesh triangulation.
                Takes explicit width and height for mesh coordinates, which are measured
                from the top left corner of the pixel coordinate system. The triangle size is
                measured in pixels.
             */
            static std::shared_ptr<extensions::puppet::PuppetT> createRectanglePuppet(uint32_t width, uint32_t height, double triangleSize);
            
            /** Applies a horizontal shift to a puppet's warper by adding handles and moving them.
                Takes min/max bounds to place handles on the current puppet geometry.
             */
            static void applyHorizontalShiftToWarper(extensions::puppet::PuppetT& puppet, 
                                                     float minX, float maxX, float minY, float maxY, 
                                                     float shiftAmountPixels);
            
            /** Helper method to iterate the puppet warper until convergence or max iterations (10).
             */
            static void iterateWarper(extensions::puppet::PuppetT& puppet);
            
            /** Helper for PuppetT shared_ptr - casts to PuppetT reference.
             */
            static void iterateWarper(std::shared_ptr<extensions::puppet::PuppetT> puppet);

            // Color and pixel utility functions
            /** A function that takes a hex RGBA value 0xRRGGBBAA and returns the
             corresponding pixel value.
             */
            static constexpr uint32_t RGBA(uint32_t v);
            
            static uint32_t colorDifference(uint32_t a, uint32_t b);
            static bool withinTolerance(uint32_t a, uint32_t b, uint32_t tolerance);
            static bool equalPixels(const std::vector<uint32_t>& pixels, const std::vector<uint32_t>& expected, uint32_t tolerance);
            
            /** Black in AABBGGRR format */
            static uint32_t blackColor(uint8_t alpha = 255);

            // Puppet geometry validation
            static void comparePoints(const optimtools::mesh_traits<double,2>::PointVector& actual, const std::vector<optimtools::mesh_traits<double,2>::Point>& expected);
            static void comparePoints(const optimtools::mesh_traits<double,2>::PointVector& actual, const optimtools::mesh_traits<double,2>::PointVector& expected);
            static void compareTriangles(const optimtools::mesh_traits<double,2>::TriangleVector& actual, const std::vector<std::vector<int>>& expected);
            static void printPuppetGeometry(const extensions::puppet::PuppetT &puppet);
            static void validatePuppetBaseGeometry(const extensions::puppet::PuppetT &puppet,
                                                   const optimtools::mesh_traits<double,2>::PointVector& expectedBaseMeshPoints);
            static void validatePuppetGeometry(const extensions::puppet::PuppetT &puppet,
                                               const std::vector<optimtools::mesh_traits<double,2>::Point>& expectedInitialPoints,
                                               const std::vector<optimtools::mesh_traits<double,2>::Point>& expectedDeformedPoints,
                                               const std::vector<std::vector<int>>& expectedTriangles);
            static void printPuppetGeometryValidation(const extensions::puppet::PuppetT &puppet,
                                                      const std::vector<optimtools::mesh_traits<double,2>::Point>& expectedInitialPoints,
                                                      const std::vector<optimtools::mesh_traits<double,2>::Point>& expectedDeformedPoints,
                                                      const std::vector<std::vector<int>>& expectedTriangles);
            static void printStrokeInfo(const geom::Vector2f& strokeStart, 
                                        const geom::Vector2f& strokeEnd,
                                        const extensions::puppet::PuppetT& puppet);

            // Drawing and layer utilities
            static void drawImage(renderer::GPUTarget& target, gpu::Texture& dst, const shapes::Image& image);
            static std::string createGuid();
            static model::LayerRef addRasterLayer(model::Drawing& drawing, uint32_t w=4, uint32_t h=4);
            static model::LayerRef addVectorLayer(model::Drawing& drawing);
            static model::LayerRef addLayerMask(model::Drawing& drawing, const model::LayerRef& layer,
                                               uint32_t w, uint32_t h, bool reveal);
            static model::LayerRef addLayerMask(model::Drawing& drawing, const model::LayerRef& layer, bool reveal);
            static void setRasterLayerPixels(const model::LayerRef& layer, const void* pixels);
            static void drawSquare(model::LayerRef layer, float xMin, float yMin, float xMax, float yMax, geom::Vector4f color);
            static void drawCircle(model::LayerRef layer, float centerX, float centerY, float radius, geom::Vector4f color);

            // Pixel data access
            static std::vector<uint32_t> getPuppetPixels(const model::LayerRef& layer);
            
            template <typename T>
            static std::vector<T> getRasterLayerData(const model::LayerRef& layer, const char* shardName);
            
            static std::vector<uint32_t> getRasterLayerPixels(const model::LayerRef& layer);
            
            static const graphics::Group& getVectorLayerRootGroup(const model::LayerRef& layer);

            // Test helper methods from PuppetFunctionTests
            static model::LayerRef createTiledDrawing(std::unique_ptr<model::Drawing>& drawing, bool targetMask, uint32_t w, uint32_t h, uint32_t tilesize = 4);
            static bool applyPuppet(model::LayerRef targetLayer, std::shared_ptr<extensions::puppet::PuppetT> puppet);
            static bool testDrawRasterLayerWithOptions(bool targetMask);
        };
    }
}

#endif  // TESTS_PUPPETTESTHELPERS
