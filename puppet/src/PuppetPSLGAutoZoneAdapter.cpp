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

#include "PuppetPSLGAutoZoneAdapter.h"

#include "PuppetPrograms.h"

#include "sketch-foundation/shapes/FigureFunctions.h"
#include "sketchlib/control/LayerColorFunctions.h"
#include "sketch/v2/src/operators/algorithms/DouglasPeuckerSimplify.h"

namespace
{
    /**
     * Calculate perimeter by summing lengths of all curve segments
     */
    float calculatePerimeter(const shapes::Path& path)
    {
        float perimeter = 0.0f;
        for (const shapes::Curve& curve : path.curves)
        {
            for (size_t i = 1; i < curve.pts.size(); ++i)
            {
                geom::Vector2f p1 = curve.pts[i - 1];
                geom::Vector2f p2 = curve.pts[i];
                float dx = p2.x - p1.x;
                float dy = p2.y - p1.y;
                perimeter += sqrt(dx * dx + dy * dy);
            }
            
            // Close the loop by connecting last point to first point
            if (curve.pts.size() > 2)
            {
                geom::Vector2f p1 = curve.pts.back();
                geom::Vector2f p2 = curve.pts.front();
                float dx = p2.x - p1.x;
                float dy = p2.y - p1.y;
                perimeter += sqrt(dx * dx + dy * dy);
            }
        }
        return perimeter;
    }

    /**
     * Calculate perimeter estimates for all zone results
     */
    std::vector<float> calculatePerimeterEstimates(const std::vector<control::LayerColorFunctions::ZoneResult>& zoneResults)
    {
        std::vector<float> perimeterEstimates;
        perimeterEstimates.reserve(zoneResults.size());
        for (const auto& zoneResult : zoneResults)
        {
            perimeterEstimates.push_back(calculatePerimeter(zoneResult.figure.path()));
        }
        return perimeterEstimates;
    }

    /**
     * Draws the source image to the scratch buffer with the given dilation
     * If invertSrc is true, the source image will be inverted before dilation
     */
    std::unique_ptr<renderer::ScratchBuffer> drawDilationToScratchBuffer(composition::EffectFullRasterSource& source,
                                                                         int level,
                                                                         uint32_t dilation,
                                                                         bool invertSrc)
    {
        // Setup GPU target
        renderer::GPUTargetFactory::LocalTarget localTarget(renderer::GPUTargetFactory::instance());
        renderer::GPUTarget& target = *localTarget;
        
        // Extract texture from EffectFullRasterSource
        std::shared_ptr<renderer::ConstVirtualTexture> texture = source.texture(target, level);
        const uint32_t width = texture->width();
        const uint32_t height = texture->height();
        
        // Create expanded scratch buffer for dilation result to avoid cropping
        const uint32_t expandedWidth = width + 2 * dilation;
        const uint32_t expandedHeight = height + 2 * dilation;
        std::unique_ptr<renderer::ScratchBuffer> scratchBuffer = std::make_unique<renderer::ScratchBuffer>();
        scratchBuffer->acquire(target, gpu::kRed, gpu::kUnsignedByte, expandedWidth, expandedHeight);
        
        // Use PuppetDilateProgramDrawer to dilate the source to scratch buffer
        renderer::ConstVirtualTexture::Read textureAccess(target, *texture);
        renderer::PuppetDilateProgramDrawer dilateDrawer;
        dilateDrawer.draw(target, 
                          dilation,
                          *textureAccess,
                          scratchBuffer->get(),
                          invertSrc);

        // Explicitly flush the dilation work
        //
        // This is typically discouraged, since we would rather queue work and only synchronize when
        // absolutely necessary, but the ensuing AutoZone logic is multi-threaded and on the CPU. This
        // means that, without flushing, AutoZone's background threads would read the scratch buffer into
        // memory before the dilation occurs. The easiest solution for synchronization here is to simply flush.
        target.renderContext().flush();

        return scratchBuffer;
    }

    /**
     * Produces a shapes::Image from a scratch buffer
     */
    shapes::Image imageFromScratchBuffer(renderer::ScratchBuffer& scratchBuffer)
    {
        // Wrap the texture in VirtualTextureArray
        std::unique_ptr<renderer::VirtualTextureArray> textureArray = 
            std::make_unique<renderer::VirtualGPUTextureArray>(scratchBuffer.get());
        
        // Create shapes::Image from VirtualTextureArray
        shapes::GPURasterImageProvider::CacheParameters params(renderer::GPUTargetFactory::instance(),
                                                               std::make_shared<renderer::ResidentTextureArrayAllocator>(512));
        shapes::Image image = shapes::GPURasterImageProvider::createImage(params, std::move(textureArray), 
                                                                          "", "", shapes::ImageOrientation::topLeft);
        
        return image;
    }

    /**
    * Extract zone figures with area estimates from a single-channel dilated image
    */
    std::vector<control::LayerColorFunctions::ZoneResult> zoneFiguresWithAreasFromImage(const shapes::Image& image)
    {
        // Define pixel comparison function for single-channel (red) dilated image
        // Background predicate - for dilated red channel, 0 = background, >0 = content
        auto pixelIsBackground = [](uint8_t p)
        {
            return p == 0;
        };

        // Pixel comparison predicate
        // Our content is binarized at this point, so we can compare the pixels directly
        auto pixelComp = [](uint8_t u, uint8_t v)
        {
            return u == v;
        };

        // Call findZoneBoundariesWithArea - returns vector of ZoneResult with figures and areas
        return control::LayerColorFunctions::findZoneBoundariesWithArea<uint8_t>(image, pixelComp, pixelIsBackground);
    }

    /**
    * Simply paths using Douglas-Peucker algorithm with the given tolerance
    */
    void simplifyPaths(std::vector<control::LayerColorFunctions::ZoneResult>& zoneResults, double outlineTolerance)
    {
        for (control::LayerColorFunctions::ZoneResult& zoneResult : zoneResults)
        {
            for (shapes::Curve& curve : zoneResult.figure.mutablePath().curves)
            {
                // Apply Douglas-Peucker simplification instead of sampling
                std::vector<geom::Vector2f> simplifiedPoints =
                    operators::algorithms::DouglasPeuckerSimplify::simplify(curve.pts, outlineTolerance);
                curve.pts = simplifiedPoints;
            }
        }
    }

    /**
     * Remove coincident segments from all figures in the zone results
     * If removing coincident segments from a curve results in fewer than three points, that curve is removed
     * If removing curves creates a zone result with no curves, the zone result is removed
     */
    void removeCoincidentSegments(std::vector<control::LayerColorFunctions::ZoneResult>& zoneResults)
    {
        // Iterate backwards since we might remove zone results
        for (int i = zoneResults.size() - 1; i >= 0; --i)
        {
            control::LayerColorFunctions::ZoneResult& zoneResult = zoneResults[i];
            
            // Remove coincident segments from the figure
            shapes::FigureFunctions::removeCoincidentSegments(zoneResult.figure);
            
            // Remove curves with fewer than 3 points since they cannot be triangulated
            shapes::Path& path = zoneResult.figure.mutablePath();
            auto it = std::remove_if(path.curves.begin(), path.curves.end(), 
                                     [](const shapes::Curve& curve)
                                     {
                                         return curve.pts.size() < 3;
                                     });
            path.curves.erase(it, path.curves.end());
            
            // Remove the zone result if no curves remain
            if (path.curves.empty())
            {
                zoneResults.erase(zoneResults.begin() + i);
            }
        }
    }

    /**
    * Create PSLGResults from the provided zone results
    * The transform will be applied to all curve points to convert them from AutoZone coordinate space to source space
    */
    std::vector<extensions::puppet::PSLGResult> PSLGResultsFromZoneResults(const std::vector<control::LayerColorFunctions::ZoneResult>& zoneResults,
                                                                           const geom::Affinity2f& transform,
                                                                           float sourceScale,
                                                                           const std::vector<float>& perimeterEstimates)
    {
        using Point = optimtools::geometry_traits<double,2>::Point;
        using PointVector = optimtools::geometry_traits<double,2>::PointVector;

        std::vector<extensions::puppet::PSLGResult> pslgResults;
        for (size_t i = 0; i < zoneResults.size(); ++i)
        {
            const control::LayerColorFunctions::ZoneResult& zoneResult = zoneResults[i];
            const shapes::Path& path = zoneResult.figure.path();

            // Calculate total points needed for this figure (across all curves)
            size_t totalPoints = 0;
            for (const shapes::Curve& curve : path.curves)
            {
                totalPoints += curve.pts.size();
            }

            optimtools::PSLG<double, int64_t> pslg(-1);  // Default vertex data of -1

            // Pre-reserve memory for all outline points
            // This is a performance optimization when there are many, point-dense curves
            pslg.reserve_outline_points(totalPoints);

            // Process each curve - each curve represents one boundary (outline or hole)
            for (const shapes::Curve& curve : path.curves)
            {
                // Convert curve points to PointVector
                PointVector points(curve.pts.size());
                for (size_t i = 0; i < curve.pts.size(); i++)
                {
                    geom::Vector2f pt = transform.transform(curve.pts[i]);
                    points[i] = Point(pt.x, pt.y);
                }

                if (points.size() >= 3)
                {
                    // Clockwise paths are treated as holes while counter clockwise
                    // paths are treated as content.
                    bool isCounterClockwise = optimtools::is_polygon_counterclockwise(points);
                    pslg.add_outline(points, /* is_hole = */ !isCounterClockwise);
                }
            }
             
            if (pslg.num_outlines() > 0)
            {
                 // Scale up the area and perimeter estimates if needed
                 pslgResults.emplace_back(std::move(pslg),
                                          sourceScale * sourceScale * zoneResult.area,
                                          sourceScale * perimeterEstimates[i]);
            }
        }
        return pslgResults;
    }
} // anonymous namespace

namespace extensions
{
    namespace puppet
    {
        std::vector<PSLGResult> PuppetPSLGAutoZoneAdapter::createPSLGs(composition::EffectFullRasterSource& source,
                                                                       int sourceLevel,
                                                                       double outlineTolerance,
                                                                       bool invertSource)
        {
            const int32_t dilation = std::ceil(outlineTolerance);

            // STEP 1: Dilate texture data to single-channel scratch buffer
            std::unique_ptr<renderer::ScratchBuffer> scratchBuffer = drawDilationToScratchBuffer(source,
                                                                                                 sourceLevel,
                                                                                                 dilation,
                                                                                                 invertSource);

            // STEP 2: Create shapes::Image from scratch buffer
            shapes::Image image = imageFromScratchBuffer(*scratchBuffer);
            
            // STEP 3: Extract zone figures with area estimates from dilated image
            std::vector<control::LayerColorFunctions::ZoneResult> zoneResults = zoneFiguresWithAreasFromImage(image);

            // STEP 4: Calculate perimeter estimates from presimplified curves
            std::vector<float> perimeterEstimates = calculatePerimeterEstimates(zoneResults);

            // STEP 5: Simplify paths using Douglas-Peucker algorithm
            simplifyPaths(zoneResults, outlineTolerance);

            // STEP 6: Remove coincident segments from all the figures in the zone results. These segments
            // tend to arise from the simplification process and can cause problems for triangulation.
            removeCoincidentSegments(zoneResults);

            // STEP 7: Create PSLGResults from simplified zone results
            // The curves received from AutoZone are in pixel coordinates relative to the image that we
            // passed in as input. We need to convert from the coordinate system of this image back to
            // the coordinate system of the source. To generate the image from the source, we applied
            // the following steps:
            //    1. Translate to the source bounds minimum so that the image is tightly bounded
            //    2. Apply a scale factor based on the source level (aka downsampling)
            //    3. Dilate the result
            //
            // Thus, to convert from the AutoZone coordinate system back to source, we apply these
            // transformations in reverse (i.e dilate translation first).
            float scale = std::pow(2, sourceLevel);  // scale up by the level that we used for the source
            const geom::Box2f& sourceBounds = source.sourceBounds();
            const geom::Affinity2f autoZoneToSourceTransform = geom::Affinity2f::translate(sourceBounds.xmin, sourceBounds.ymin) *
                                                               geom::Affinity2f::scale(scale, scale) *
                                                               geom::Affinity2f::translate(-dilation, -dilation);
            return PSLGResultsFromZoneResults(zoneResults, autoZoneToSourceTransform, scale, perimeterEstimates);
        }

    }
}
