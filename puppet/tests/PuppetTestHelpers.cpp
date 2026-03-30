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

#include "PuppetTestHelpers.h"

#include <algorithm>
#include <cmath>

#include "CPTestCase.h"
#include "composition/Composition"

#include "sketchlib/control/DrawingInitialization.h"
#include "sketchlib/control/StrokeSample.h"
#include "sketchlib/model/Drawing.h"
#include "sketchlib/model/ImageDataProvider.h"
#include "sketchlib/model/LayerGroupContents.h"
#include "sketchlib/model/MaskAreaSnapshot.h"
#include "sketchlib/model/RasterLayerContents.h"
#include "sketchlib/model/VectorLayerContents.h"

#include "optimtools/all.h"
#include "optimtools_extras/mesh/all.h"
#include "puppet_warp/puppet_from_bitmask.h"
#include "AbstractPuppet.h"
#include "PuppetEffect.h"
#include "api/control/PuppetFunctions.h"

namespace extensions
{
    namespace puppet
    {
        using namespace optimtools;
        
        // Puppet creation and manipulation
        std::shared_ptr<extensions::puppet::PuppetT> PuppetTestHelpers::createRectanglePuppet(uint32_t width, uint32_t height, double triangleSize)
        {
            USING_OPTIMTOOLS_MESH_TYPES(typename, double, 2);
            
            // Create a simple rectangle figure using actual pixel coordinates
            optimtools::PSLG<double, optimtools::Int> pslg(static_cast<optimtools::Int>(-1));
            PointVector rectangleOutline(4);  // 4 points for rectangle corners
            rectangleOutline[0] = Point(0.0, 0.0);
            rectangleOutline[1] = Point(width, 0.0);
            rectangleOutline[2] = Point(width, height);
            rectangleOutline[3] = Point(0.0, height);
            pslg.add_outline(rectangleOutline);
            
            // Create the figure
            optimtools::PuppetFigure<double, optimtools::PuppetHandle> figure(pslg);
            
            // Create and return the puppet (as PuppetT but internally it's SculptingPuppet)
            shared_ptr<SculptingPuppet<double>> puppet = std::make_shared<SculptingPuppet<double>>(figure);
            // Note: auto-pin is disabled for tests by passing false to add_handle() in test methods
            
            // set test puppet parameters so user defaults can change without affecting tests
            puppet->set_triangle_size(triangleSize);
            puppet->set_bird_s(1);
            puppet->set_bird_w(1);
            puppet->set_max_iterations(200);

            // Make sure the puppet is fully initialized
            puppet->iterate_warper();

            return puppet;
        }
        
        void PuppetTestHelpers::applyHorizontalShiftToWarper(extensions::puppet::PuppetT& puppet, 
                                                            float minX, float maxX, float minY, float maxY, 
                                                            float shiftAmountPixels)
        {
            USING_OPTIMTOOLS_MESH_TYPES(typename, double, 2);
            
            // Add handles at all four corner
            // Note: the latest handle-selection logic makes it possible to click slightly outside a mesh.
            // Here, we could click slightly outside the mesh to make sure we grab the intended vertex.
            // Note: we currently don't support multi touch puppet manipulation in the app

            float handleThreshold = 0.01; // find handle within this radius
            // Disable auto-pin for tests (autoPinEnabled=false, autoPinFraction=0.0)
            AbstractPuppet::HandleDragger handleBottomLeft = puppet.add_handle(Point(minX, minY), handleThreshold, false, 0.0);  // bottom-left
            AbstractPuppet::HandleDragger handleBottomRight = puppet.add_handle(Point(maxX, minY), handleThreshold, false, 0.0); // bottom-right
            AbstractPuppet::HandleDragger handleTopLeft = puppet.add_handle(Point(minX, maxY), handleThreshold, false, 0.0);     // top-left
            AbstractPuppet::HandleDragger handleTopRight = puppet.add_handle(Point(maxX, maxY), handleThreshold, false, 0.0);    // top-right
            
            // Move all handles to create uniform horizontal shift by shift_amount_pixels
            puppet.move_dragger(handleBottomLeft, Point(minX + shiftAmountPixels, minY));
            puppet.move_dragger(handleBottomRight, Point(maxX + shiftAmountPixels, minY));
            puppet.move_dragger(handleTopLeft, Point(minX + shiftAmountPixels, maxY));
            puppet.move_dragger(handleTopRight, Point(maxX + shiftAmountPixels, maxY));
            
            // Run iterations to deform the mesh
            iterateWarper(puppet);
        }
        
        void PuppetTestHelpers::iterateWarper(extensions::puppet::PuppetT& puppet)
        {
            // Run iterations to deform the mesh
            for (int i = 0; i < 10; ++i) {
                if (puppet.iterate_warper())
                    break;
            }
        }
        
        void PuppetTestHelpers::iterateWarper(std::shared_ptr<extensions::puppet::PuppetT> puppet)
        {
            iterateWarper(*puppet);
        }

        // Color and pixel utility functions
        constexpr uint32_t PuppetTestHelpers::RGBA(uint32_t v)
        {
            return (((v & 0xFF000000) >> 24) |
                    ((v & 0x00FF0000) >> 8) |
                    ((v & 0x0000FF00) << 8) |
                    ((v & 0x000000FF) << 24));
        }
        
        uint32_t PuppetTestHelpers::colorDifference(uint32_t a, uint32_t b) {
            int32_t a1 = (a >> 24) & 0xFF;
            int32_t a2 = (a >> 16) & 0xFF;
            int32_t a3 = (a >> 8)  & 0xFF;
            int32_t a4 =  a        & 0xFF;
            
            int32_t b1 = (b >> 24) & 0xFF;
            int32_t b2 = (b >> 16) & 0xFF;
            int32_t b3 = (b >> 8)  & 0xFF;
            int32_t b4 =  b        & 0xFF;
            
            return std::abs(a1 - b1) +
                std::abs(a2 - b2) +
                std::abs(a3 - b3) +
                std::abs(a4 - b4);
        }
        
        bool PuppetTestHelpers::withinTolerance(uint32_t a, uint32_t b, uint32_t tolerance) {
            uint32_t diff = colorDifference(a, b);
            if (diff <= tolerance)
                return true;
            else
                return false;
        }
        
        bool PuppetTestHelpers::equalPixels(const std::vector<uint32_t>& pixels, const std::vector<uint32_t>& expected, uint32_t tolerance) {
            if (pixels.size() != expected.size())
                return false;
            
            return std::equal(pixels.begin(), pixels.end(), expected.begin(),
                              [tolerance](uint32_t a, uint32_t b) { return withinTolerance(a, b, tolerance); });
        }
        
        uint32_t PuppetTestHelpers::blackColor(uint8_t alpha)
        {
            return (alpha << 24) | 0x000000;
        }

        // Puppet geometry validation
        void PuppetTestHelpers::comparePoints(const optimtools::mesh_traits<double,2>::PointVector& actual, const std::vector<optimtools::mesh_traits<double,2>::Point>& expected)
        {
            constexpr double epsilon = 0.1;
            CPAssertTrue(actual.size() == expected.size());
            for (size_t i = 0; i < actual.size(); ++i) {
                const auto& actualPoint = actual[i];
                const auto& expectedPoint = expected[i];
                CPAssertTrue(std::abs(actualPoint.x() - expectedPoint.x()) < epsilon);
                CPAssertTrue(std::abs(actualPoint.y() - expectedPoint.y()) < epsilon);
            }
        }
        
        void PuppetTestHelpers::comparePoints(const optimtools::mesh_traits<double,2>::PointVector& actual, const optimtools::mesh_traits<double,2>::PointVector& expected)
        {
            constexpr double epsilon = 0.1;
            CPAssertTrue(actual.size() == expected.size());
            for (size_t i = 0; i < actual.size(); ++i) {
                const auto& actualPoint = actual[i];
                const auto& expectedPoint = expected[i];
                CPAssertTrue(std::abs(actualPoint.x() - expectedPoint.x()) < epsilon);
                CPAssertTrue(std::abs(actualPoint.y() - expectedPoint.y()) < epsilon);
            }
        }
        
        void PuppetTestHelpers::compareTriangles(const optimtools::mesh_traits<double,2>::TriangleVector& actual, const std::vector<std::vector<int>>& expected)
        {
            CPAssertTrue(actual.size() == expected.size());
            for (size_t i = 0; i < actual.size(); ++i) {
                for (size_t j = 0; j < 3; ++j) {
                    const auto& actualValue = actual[i][j];
                    const auto& expectedValue = expected[i][j];
                    CPAssertTrue(actualValue == expectedValue);
                }
            }
        }
        
        void PuppetTestHelpers::printPuppetGeometry(const extensions::puppet::PuppetT &puppet)
        {
            USING_OPTIMTOOLS_MESH_TYPES(typename,double,2);
            
            printf("\nPuppetGeometry:\n");
            const TriangleVector& triangles = puppet.base_mesh().triangles();
            const PointVector& initialPoints = puppet.initial_points();
            const PointVector& deformedPoints = puppet.deformed_mesh().points();
            const IntVector& pointHandles = puppet.base_mesh().point_handles();
            
            printf("std::vector<Point> expectedInitialPoints = {\n");
            for (size_t i = 0; i < initialPoints.size(); ++i) {
                printf("    Point(%.1f, %.1f)", initialPoints[i].x(), initialPoints[i].y());
                if (i < initialPoints.size() - 1) {
                    printf(",");
                }
                printf("\n");
            }
            printf("};\n");
            
            printf("std::vector<Point> expectedDeformedPoints = {\n");
            for (size_t i = 0; i < deformedPoints.size(); ++i) {
                printf("    Point(%.1f, %.1f)", deformedPoints[i].x(), deformedPoints[i].y());
                if (i < deformedPoints.size() - 1) {
                    printf(",");
                }
                printf("\n");
            }
            printf("};\n");
            
            printf("std::vector<std::vector<int>> expectedTriangles = {\n");
            for (size_t i = 0; i < triangles.size(); ++i) {
                printf("    {%lld, %lld, %lld}", triangles[i][0], triangles[i][1], triangles[i][2]);
                if (i < triangles.size() - 1) {
                    printf(",");
                }
                printf("\n");
            }
            printf("};\n");
            
            printf("std::vector<int> expectedPointHandles = {\n");
            for (size_t i = 0; i < pointHandles.size(); ++i) {
                printf("    %lld", pointHandles[i]);
                if (i < pointHandles.size() - 1) {
                    printf(",");
                }
                printf("\n");
            }
            printf("};\n");
        }
        
        void PuppetTestHelpers::validatePuppetBaseGeometry(const extensions::puppet::PuppetT &puppet,
                                                           const optimtools::mesh_traits<double,2>::PointVector& expectedBaseMeshPoints)
        {
            USING_OPTIMTOOLS_MESH_TYPES(typename,double,2);
            
            const PointVector& baseMeshPoints = puppet.base_mesh().points();
            comparePoints(baseMeshPoints, expectedBaseMeshPoints);
        }
        
        void PuppetTestHelpers::validatePuppetGeometry(const extensions::puppet::PuppetT &puppet,
                                                       const std::vector<optimtools::mesh_traits<double,2>::Point>& expectedInitialPoints,
                                                       const std::vector<optimtools::mesh_traits<double,2>::Point>& expectedDeformedPoints,
                                                       const std::vector<std::vector<int>>& expectedTriangles)
        {
            USING_OPTIMTOOLS_MESH_TYPES(typename,double,2);
            
            const TriangleVector& triangles = puppet.base_mesh().triangles();
            const PointVector& initialPoints = puppet.initial_points();
            const PointVector& deformedPoints = puppet.deformed_mesh().points();
            
            // Enable printPuppetGeometry for debugging
//            printPuppetGeometry(puppet); // for debugging
            
            comparePoints(deformedPoints, expectedDeformedPoints);
            comparePoints(initialPoints, expectedInitialPoints);
            compareTriangles(triangles, expectedTriangles);
        }
        
        void PuppetTestHelpers::printPuppetGeometryValidation(const extensions::puppet::PuppetT &puppet,
                                                             const std::vector<optimtools::mesh_traits<double,2>::Point>& expectedInitialPoints,
                                                             const std::vector<optimtools::mesh_traits<double,2>::Point>& expectedDeformedPoints,
                                                             const std::vector<std::vector<int>>& expectedTriangles)
        {
            USING_OPTIMTOOLS_MESH_TYPES(typename,double,2);
            
            const PointVector& actualInitialPoints = puppet.initial_points();
            const PointVector& actualDeformedPoints = puppet.deformed_mesh().points();
            const TriangleVector& actualTriangles = puppet.base_mesh().triangles();
            
            std::cout << "\nInitial Points:" << std::endl;
            for (size_t i = 0; i < expectedInitialPoints.size(); ++i) {
                std::cout << "  [" << i << "] (" << expectedInitialPoints[i][0] << ", " << expectedInitialPoints[i][1] << ")" << std::endl;
            }
            
            std::cout << "Expected Deformed Points:" << std::endl;
            for (size_t i = 0; i < expectedDeformedPoints.size(); ++i) {
                std::cout << "  [" << i << "] (" << expectedDeformedPoints[i][0] << ", " << expectedDeformedPoints[i][1] << ")" << std::endl;
            }
            
            std::cout << "ACTUAL Deformed Points:" << std::endl;
            for (size_t i = 0; i < actualDeformedPoints.size(); ++i) {
                std::cout << "  [" << i << "] (" << actualDeformedPoints[i].x() << ", " << actualDeformedPoints[i].y() << ")" << std::endl;
            }
            
            std::cout << "Expected Triangles:" << std::endl;
            for (size_t i = 0; i < expectedTriangles.size(); ++i) {
                std::cout << "  [" << i << "] {" << expectedTriangles[i][0] << ", " << expectedTriangles[i][1] << ", " << expectedTriangles[i][2] << "}" << std::endl;
            }
            
            std::cout << "Actual Triangles:" << std::endl;
            for (size_t i = 0; i < actualTriangles.size(); ++i) {
                std::cout << "  [" << i << "] {" << actualTriangles[i][0] << ", " << actualTriangles[i][1] << ", " << actualTriangles[i][2] << "}" << std::endl;
            }
        }
        
        void PuppetTestHelpers::printStrokeInfo(const geom::Vector2f& strokeStart, const geom::Vector2f& strokeEnd,
                                               const extensions::puppet::PuppetT& puppet)
        {
            USING_OPTIMTOOLS_MESH_TYPES(typename,double,2);
            
            const PointVector& deformedPoints = puppet.deformed_mesh().points();
            
            std::cout << "\nStroke: (" << strokeStart.x << ", " << strokeStart.y << ") -> (" 
                     << strokeEnd.x << ", " << strokeEnd.y << ")" << std::endl;
            
            std::cout << "Distances from stroke start to vertices:" << std::endl;
            for (size_t i = 0; i < deformedPoints.size(); ++i) {
                float dx = strokeStart.x - deformedPoints[i].x();
                float dy = strokeStart.y - deformedPoints[i].y();
                float distance = std::sqrt(dx*dx + dy*dy);
                std::cout << "  [" << i << "] distance = " << distance 
                         << " from (" << deformedPoints[i].x() << ", " << deformedPoints[i].y() << ")" << std::endl;
            }
        }

        // Drawing and layer utilities
        void PuppetTestHelpers::drawImage(renderer::GPUTarget& target, gpu::Texture& dst, const shapes::Image& image)
        {
            class Visitor : public shapes::ImageProvider::DataVisitor
            {
                renderer::TexturedTextureTarget& _dstTarget;
                const geom::Affinity2f& _transform;
            public:
                Visitor(renderer::TexturedTextureTarget& dstTarget,
                        const geom::Affinity2f& transform) :
                _dstTarget(dstTarget),
                _transform(transform)
                {
                }
                void visitProvider(const shapes::ImageProvider& p, const shapes::ImageProvider::GPURasterData& data) override
                {
                    data.draw(_dstTarget,
                              geom::Projectivity2f(_transform),
                              geom::Box2f(0,1,0,1),
                              renderer::GPUBlitCompositor(),
                              1.0,
                              renderer::GPUCompositingOptions().replace(true));
                }
            };
            
            gpu::RenderContext& ctx = target.renderContext();
            gpu::AutoSaveState saveState(ctx);
            saveState.saveColorBuffer();
            saveState.saveScissor();
            
            // Set up and bind a target for rendering to the texture
            renderer::TexturedTextureTarget dstTarget(target, &dst);
            dstTarget.bind();
            
            ctx.disableScissor();
            ctx.clearColorBuffer(0, 0, 0, 0);
            
            // Transform must map image bounds to cover NDC space
            const geom::Affinity2f transform(2.0/image.width(), 0, -1.0,
                                             0, 2.0/image.height(), -1.0);
            
            // Visit image to draw it
            Visitor visitor(dstTarget, transform);
            shapes::ImageProvider::Access(image.provider()).visit(visitor);
        }
        
        std::string PuppetTestHelpers::createGuid()
        {
            return control::DrawingInitialization::drawingConfig().generateGUID().to_string();
        }
        
        model::LayerRef PuppetTestHelpers::addRasterLayer(model::Drawing& drawing, uint32_t w, uint32_t h)
        {
            // Add raster layer to root
            auto rootContents = drawing.root()->contents()->as<model::LayerGroupContents>();
            rootContents->insertLayer(model::RasterLayerContents::createLayer(drawing, "1", w, h), 0);
            model::LayerRef rasterLayer = rootContents->at(0);
            return rasterLayer;
        }
        
        model::LayerRef PuppetTestHelpers::addVectorLayer(model::Drawing& drawing)
        {
            // Add raster layer to root
            auto rootContents = drawing.root()->contents()->as<model::LayerGroupContents>();
            rootContents->insertLayer(model::VectorLayerContents::createLayer(drawing, "1"), 0);
            model::LayerRef vectorLayer = rootContents->at(0);
            return vectorLayer;
        }
        
        model::LayerRef PuppetTestHelpers::addLayerMask(model::Drawing& drawing, const model::LayerRef& layer,
                                                       uint32_t w, uint32_t h, bool reveal)
        {
            layer->setRasterMask(model::RasterLayerContents::createLayer(drawing, createGuid(), w, h));
            model::LayerRef mask = layer->rasterMask();
            mask->setMaskMode(reveal ? model::MaskMode::reveal : model::MaskMode::hide);
            return mask;
        }

        model::LayerRef PuppetTestHelpers::addLayerMask(model::Drawing& drawing, const model::LayerRef& layer, bool reveal)
        {
            return addLayerMask(drawing, layer, 4, 4, reveal);
        }
        
        void PuppetTestHelpers::setRasterLayerPixels(const model::LayerRef& layer, const void* pixels)
        {
            auto rasterContents = layer->contents()->as<model::RasterLayerContents>();
            const int32_t w = rasterContents->pixelWidth();
            const int32_t h = rasterContents->pixelHeight();
            rasterContents->setPixels(model::createRasterLayerDataProvider({"color"},
                                                                           [=](const std::string& shardName)
                                                                           {
                return model::createImageDataProvider(model::ImageDataProvider::Format::RGBAU8, w, h, pixels);
            }));
            
            // make sure the operation is complete before returning as the pixel
            // data might be on the stack
            layer->owner().flushOutput().check();
        }
        
        void PuppetTestHelpers::drawSquare(model::LayerRef layer, float xMin, float yMin, float xMax, float yMax, geom::Vector4f color)
        {
            if (layer->contents()->is<model::RasterLayerContents>())
            {
                layer->contents()->as<model::RasterLayerContents>()->node().layer().context().fillRect(xMin, yMin, xMax-xMin, yMax-yMin, color);
                layer->contents()->as<model::RasterLayerContents>()->node().layer().update();
            }
            else if (layer->contents()->is<model::VectorLayerContents>())
            {
                auto vectorContents = layer->contents()->as<model::VectorLayerContents>();
                graphics::Group& rootGroup = vectorContents->contents().mutableRootGroup();
                
                shapes::Figure f;
                shapes::Paint& paint = f.mutablePaint();
                shapes::Path& path = f.mutablePath();
                paint.assign(shapes::FillLayer(color));
                
                path.moveTo(geom::Vector2f(xMin, yMin));
                path.lineTo(geom::Vector2f(xMax, yMin));
                path.lineTo(geom::Vector2f(xMax, yMax));
                path.lineTo(geom::Vector2f(xMin, yMax));
                path.lineTo(geom::Vector2f(xMin, yMin));
                
                graphics::FigureGraphicRef graphic(graphics::FigureGraphic::create(0));
                graphic->setFigure(f);
                
                rootGroup.addChild(*graphic);
            }
        }
        
        void PuppetTestHelpers::drawCircle(model::LayerRef layer, float centerX, float centerY, float radius, geom::Vector4f color)
        {
            // Create circular figure using the built-in addCircle method
            shapes::Figure figure;
            shapes::Paint& paint = figure.mutablePaint();
            shapes::Path& path = figure.mutablePath();
            paint.assign(shapes::FillLayer(color));
            
            // Create transform to position and scale the unit circle
            // addCircle creates a unit circle, so we need to scale by radius and translate to center
            geom::Affinity2f transform(radius, 0, centerX,
                                     0, radius, centerY);
            
            // Add the circle using the built-in method
            path.addCircle(transform);
            
            if (layer->contents()->is<model::RasterLayerContents>())
            {
                // For raster layers, draw the figure to the context
                auto rasterContents = layer->contents()->as<model::RasterLayerContents>();
                auto& context = rasterContents->node().layer().context();
                context.draw(figure);
                rasterContents->node().layer().update();
            }
            else if (layer->contents()->is<model::VectorLayerContents>())
            {
                // For vector layers, add the figure to the root group
                auto vectorContents = layer->contents()->as<model::VectorLayerContents>();
                graphics::Group& rootGroup = vectorContents->contents().mutableRootGroup();
                
                graphics::FigureGraphicRef graphic(graphics::FigureGraphic::create(0));
                graphic->setFigure(figure);
                
                rootGroup.addChild(*graphic);
            }
        }

        // Pixel data access
        std::vector<uint32_t> PuppetTestHelpers::getPuppetPixels(const model::LayerRef& layer)
        {
            auto& rasterNode = layer->node().as<composition::DrawingLayerNode>();
            auto& effect = rasterNode.effect()->as<extensions::puppet::PuppetEffect>();

            renderer::GPUTargetFactory::LocalTarget target(renderer::GPUTargetFactory::instance());

            base::JobGroup group;
            effect.update(group);
            group.wait();

            shapes::Image image = effect.image();

            const uint32_t width = image.width();
            const uint32_t height = image.height();

            auto resultTexture = (*target).renderContext().createTexture(width, height, nullptr);
            drawImage(*target, *resultTexture, image);

            // Use debugTexture to visualize the texture
            gpu::debugTexture((*target).renderContext(), *resultTexture);

            // Still need to get the actual pixel data for the test
            std::vector<uint32_t> result(width*height);
            resultTexture->getPixels((*target).renderContext(), 0, 0, width, height, result.data());

            gpu::destroy((*target).renderContext(), resultTexture);
            return result;
        }
        
        template <typename T>
        std::vector<T> PuppetTestHelpers::getRasterLayerData(const model::LayerRef& layer, const char* shardName)
        {
            auto rasterContents = layer->contents()->as<model::RasterLayerContents>();

            std::vector<T> result;
            const size_t dstysize = rasterContents->pixelWidth();
            result.resize(dstysize * rasterContents->pixelHeight());
            
            rasterContents->getPixels([&](const model::RasterLayerContents::DataProvider& provider)
            {
                // grab the color shard
                auto slab = provider.getSlab(0);
                auto shard = slab->getShard(shardName);
                for (int32_t j = 0; j < shard->jsize(); ++j)
                {
                    for (int32_t i = 0; i < shard->isize(); ++i)
                    {
                        const uint32_t dstx = i * shard->tilexsize();
                        const uint32_t dsty = j * shard->tileysize();
                        T* dstptr = result.data() + dstx + dsty * dstysize;

                        // read contents of tile
                        auto data = shard->data(i, j);
                        for (int32_t y = 0; y < data->ysize(); ++y)
                        {
                            data->readRows(dstptr, sizeof(T), sizeof(T)*data->xsize(), 1);
                            dstptr += dstysize;
                        }
                    }
                }
            });
            
            return result;
        }
        
        std::vector<uint32_t> PuppetTestHelpers::getRasterLayerPixels(const model::LayerRef& layer)
        {
            return getRasterLayerData<uint32_t>(layer, "color");
        }
    
        const graphics::Group& PuppetTestHelpers::getVectorLayerRootGroup(const model::LayerRef& layer)
        {
            auto vectorContents = layer->contents()->as<model::VectorLayerContents>();
            return vectorContents->contents().rootGroup();
        }

        // Test helper methods from PuppetFunctionTests
        model::LayerRef PuppetTestHelpers::createTiledDrawing(std::unique_ptr<model::Drawing>& drawing, bool targetMask, uint32_t w, uint32_t h, uint32_t tilesize)
        {
            // Make a drawing with tile size
            drawing = model::Drawing::createForTest(tilesize);
            
            // Add raster layer to root with puppet effect
            model::LayerRef targetLayer = addRasterLayer(*drawing, w, h);
            if (targetMask)
                targetLayer = addLayerMask(*drawing, targetLayer, w, h, true);
            
            // Draw a transparent background
            drawSquare(targetLayer, 0, 0, w, h, geom::Vector4f(0,0,0,0));

            return targetLayer;
        }

        bool PuppetTestHelpers::applyPuppet(model::LayerRef targetLayer, std::shared_ptr<extensions::puppet::PuppetT> puppet)
        {
            if (!puppet)
                return false;
            
            // Get the effect and set the puppet
            auto &effect = targetLayer->node().effect()->as<extensions::puppet::PuppetEffect>();
            
            // Set the puppet as the effect's puppet
            effect.setPuppet(puppet);
            
            // Apply effect - this removes the effect and applies the transformation to the pixel data
            control::PuppetFunctions::applyEffect(targetLayer, false /*removeFromJournal*/);
            
            // The effect is removed after applying
            CPAssertTrue(targetLayer->node().effect() == nullptr);
            
            return true;
        }

        bool PuppetTestHelpers::testDrawRasterLayerWithOptions(bool targetMask)
        {
            const uint32_t w = 4;
            const uint32_t h = 4;
            
            // Make a drawing
            std::unique_ptr<model::Drawing> drawing = model::Drawing::createForTest();
            
            // Add raster layer to root with puppet effect
            model::LayerRef targetLayer = addRasterLayer(*drawing, w, h);
            if (targetMask)
                targetLayer = addLayerMask(*drawing, targetLayer, true);
            
            // Draw black square
            geom::Vector4f black(0,0,0,1);
            drawSquare(targetLayer, 0, 0, 2, 2, black);
            
            auto puppetSettings = model::PuppetSettings();
            auto defaultProgress = base::Progress();
            auto puppetEffect = control::PuppetFunctions::createEffect(targetLayer, puppetSettings, defaultProgress);
            control::PuppetFunctions::initializeEffect(puppetEffect, puppetSettings, defaultProgress);
            control::PuppetFunctions::attachEffect(targetLayer, puppetEffect);
            
            auto& node = targetLayer->node().as<composition::DrawingLayerNode>();
            auto& effect = node.effect()->as<extensions::puppet::PuppetEffect>();

            // Check the effect result before manipulating puppet
            {
                std::vector<uint32_t> expectedValues(w*h);
                expectedValues[0] = blackColor();
                expectedValues[1] = blackColor();
                expectedValues[4] = blackColor();
                expectedValues[5] = blackColor();
                
                const bool equal = equalPixels(getPuppetPixels(targetLayer), expectedValues, 0);
                if (!equal)
                    return false;
            }
            
            // Apply horizontal shift using helper method
            {
                uint32_t looseTolerance = 10; // More tolerance for horizontal shift
                const float horizontalShiftPixels = 1.0f; // 1 pixel shift to the right
                
                // Create rectangle puppet and apply horizontal shift
                auto shiftPuppet = createRectanglePuppet(2, 2, 2);
                CPAssertTrue(shiftPuppet != nullptr);
                applyHorizontalShiftToWarper(*shiftPuppet, 0, 2, 0, 2, horizontalShiftPixels);

                // Set the puppet as the effect's puppet (don't apply, just test the effect)
                auto& node = targetLayer->node().as<composition::DrawingLayerNode>();
                auto& effect = node.effect()->as<extensions::puppet::PuppetEffect>();
                effect.setPuppet(shiftPuppet);

                // Expected values for 1-pixel horizontal shift to the right
                // Original black square was at (0,0)-(2,2), shifted becomes approximately (1,0)-(3,2)
                std::vector<uint32_t> expectedValues(w*h);
                // Row 0: should have black starting at x=1
                expectedValues[0] = blackColor(0);   // (0,0) - transparent
                expectedValues[1] = blackColor(255); // (1,0) - shifted black
                expectedValues[2] = blackColor(255); // (2,0) - black 
                expectedValues[3] = blackColor(0);   // (3,0) - transparent
                // Row 1: similar pattern
                expectedValues[4] = blackColor(0);   // (0,1) - transparent
                expectedValues[5] = blackColor(255); // (1,1) - shifted black
                expectedValues[6] = blackColor(255); // (2,1) - black
                expectedValues[7] = blackColor(0);   // (3,1) - transparent
                // Rows 2,3: transparent
                for (int i = 8; i < 16; ++i) {
                    expectedValues[i] = blackColor(0);
                }
                
                // Compare to the expected values
                std::vector<uint32_t> actualPixels = getPuppetPixels(targetLayer);
                const bool equal = equalPixels(actualPixels, expectedValues, looseTolerance);
                if (!equal)
                    return false;
            }
            
            return true;
        }

        // Explicit template instantiation for common types
        template std::vector<uint32_t> PuppetTestHelpers::getRasterLayerData<uint32_t>(const model::LayerRef& layer, const char* shardName);
    }
}
