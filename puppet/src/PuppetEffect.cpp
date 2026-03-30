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

#include <array>

#include "optimtools/all.h"
#include "optimtools_extras/image/all.h"
#include "optimtools_extras/mesh/all.h"

#include "puppet_warp/puppet_from_bitmask.h"

#include "SculptingPuppet.h"

#include "base/Base"
#include "composition/Composition"
#include "composition/effects/EffectFullRasterSource.h"
#include "numerics/Numerics"

#include "PuppetEffect.h"
#include "PuppetPSLGAutoZoneAdapter.h"

#if 0
#define LOGMSG(msg) std::cout << msg << std::endl
#else
#define LOGMSG(msg)
#endif

namespace extensions
{
    namespace puppet
    {
        using namespace optimtools;

        /**
         * A utility class to store the options available in creating a puppet from
         * an image.
         */
        class PuppetFromImageOptions {
        public:
            using Point = Eigen::Array<extensions::puppet::Scalar,2,1>;
            
            PuppetFromImageOptions() :
                m_extra_dilation(0),
                m_threshold_alpha(0),
                m_image_origin(0,0),
                m_is_add_holes(true)
            { }

            Int extra_dilation() const
            {
                return m_extra_dilation;
            }

            Int& extra_dilation()
            {
                return m_extra_dilation;
            }

            Int threshold_alpha() const
            {
                return m_threshold_alpha;
            }

            Int& threshold_alpha()
            {
                return m_threshold_alpha;
            }

            Point image_origin() const {
                return m_image_origin;
            }
            
            Point& image_origin() {
                return m_image_origin;
            }
            
            bool is_add_holes() const
            {
                return m_is_add_holes;
            }

            bool& is_add_holes()
            {
                return m_is_add_holes;
            }

        private:
            Int m_extra_dilation;
            Int m_threshold_alpha;
            Point m_image_origin;
            bool m_is_add_holes;
        };


        class PuppetImage
        {
            const Int _xsize;
            const Int _ysize;
            const uint8_t* _data;
        public:
            PuppetImage(Int xsize, Int ysize, const uint8_t* data) :
                _xsize(xsize), _ysize(ysize), _data(data)
            {
            }

            Int rows() const
            {
                return _xsize;
            }
            Int cols() const
            {
                return _ysize;
            }

            // Assumed to be RGBA
            const uint8_t* operator()(Int x, Int y) const
            {
                return _data + (x + (_ysize - y - 1) * _xsize) * 4;
            }
        };

    
        Scalar default_outline_accuracy(Int image_width, Int image_height)
        {
            Int image_span = std::max(image_width, image_height);
            return std::max(Scalar(1), image_span / Scalar(1200));
        }

        /**
         * Debug function to print mesh data for debugging purposes
         */
        void printMeshDebugInfo(const PuppetT& puppet,
                               const std::vector<uint16_t>& combinedIBuffer,
                               const std::vector<float>& combinedVBuffer)
        {
            USING_OPTIMTOOLS_MESH_TYPES(typename,Scalar,2);
            
            const TriangleVector& triangles = puppet.base_mesh().triangles();
            const PointVector& srcpts = puppet.initial_points();
            const PointVector& dstpts = puppet.deformed_mesh().points();

            // Print triangles
            printf("Triangles (%zu):\n", triangles.size());
            for (size_t i = 0; i < triangles.size(); ++i)
            {
                const Triangle& tri = triangles[i];
                printf("  Triangle[%zu]: [%lld, %lld, %lld]\n", i, tri[0], tri[1], tri[2]);
            }

            // Print srcpts
            printf("Srcpts (%zu):\n", srcpts.size());
            for (size_t i = 0; i < srcpts.size(); ++i)
            {
                const Point& src = srcpts[i];
                printf("  Srcpt[%zu]: (%.6f, %.6f)\n", i, src.x(), src.y());
            }

            // Print dstpts
            printf("Dstpts (%zu):\n", dstpts.size());
            for (size_t i = 0; i < dstpts.size(); ++i)
            {
                const Point& dst = dstpts[i];
                printf("  Dstpt[%zu]: (%.6f, %.6f)\n", i, dst.x(), dst.y());
            }

            // Print combinedIBuffer
            printf("CombinedIBuffer (%zu):\n", combinedIBuffer.size());
            for (size_t i = 0; i < combinedIBuffer.size(); ++i)
            {
                printf("  Index[%zu]: %d\n", i, combinedIBuffer[i]);
            }

            // Print combinedVBuffer
            printf("CombinedVBuffer (%zu):\n", combinedVBuffer.size());
            for (size_t i = 0; i < combinedVBuffer.size(); i += 4)
            {
                printf("  Vertex[%zu]: dst(%.6f, %.6f) src(%.6f, %.6f)\n",
                       i/4, combinedVBuffer[i], combinedVBuffer[i+1],
                       combinedVBuffer[i+2], combinedVBuffer[i+3]);
            }
        }

        /**
         * Creates PSLGs from the non-transparent regions in a PNG image.
         * Returns all connected components found in the image.
         * Checks progress and may throw an exception if the operation is cancelled.
         */
        std::vector<PSLG<Scalar,Int>> create_image_puppet_pslgs(const PuppetImage* puppet_image, const PuppetFromImageOptions& options,
                                                                const base::Progress& progress, bool treatAlphaAsInverted)
        {
            USING_OPTIMTOOLS_MESH_TYPES(typename,Scalar,2);

            // Determine some parameters we'll need to create a nice simplified outline.
            double outline_accuracy = default_outline_accuracy(puppet_image->rows(), puppet_image->cols());
            Int mask_dilation;
            if (options.extra_dilation() >= 0)
            {
                // Dilate enough that the outline won't overlap the opaque region of
                // the image, plus a bit extra if requested (which can be necessary
                // if the image is to be used as a texture with bilinear or bicubic
                // interpolation).
                mask_dilation = options.extra_dilation() + (Int)ceil(outline_accuracy);
            } else
            {
                // Disable dilation.
                mask_dilation = 0;
            }
            progress.throwIfCancelled();
            LOGMSG("image outline accuracy will be " << outline_accuracy << " pixels");
            LOGMSG("image dilation is " << mask_dilation << " pixels");

            // Create a binary mask based on the alpha channel of the image.
            // The requirements of `outline_bitmask` mean that we need to create
            // a fringe of width at least `1+mask_dilation` to ensure at least one
            // `false` mask pixel around the image after dilation.
            BitArray mask(puppet_image->rows(), puppet_image->cols(), 1+mask_dilation);
            {
#if LOG_SCULPTING_PERFORMANCE
                base::PerformanceTimer timer("Sculpting: segmenting Layer to PSLGs\n\tSculpting: binary mask");
#endif
                // When inverting, we effectively want 255 to be transparent and 0 to be opaque, so
                // we treat the pixel value as p = 255 - p. Note that we intentionally do not pass
                // in the 'treatAlphaAsInverted' bool into this lambda, since it would be inefficient to check
                // it on a per-pixel basis.
                if (treatAlphaAsInverted)
                {
                    mask.set_from([&](Int x, Int y)
                    {
                        return (255 - (*puppet_image)(x,y)[3]) > options.threshold_alpha();
                    });
                }
                else
                {
                    mask.set_from([&](Int x, Int y)
                    {
                        return (*puppet_image)(x,y)[3] > options.threshold_alpha();
                    });
                }
                LOGMSG("initialized mask");
            }
            progress.throwIfCancelled();

            // Dilating the mask ensures that the outline never crosses inside the
            // boundrary of the opaque region of the image.  If you're not using the
            // image as a texture this may not be necessary, in which case you can
            // set `extra_dilation` to a negative number to disble it.
            if (options.extra_dilation() >= 0)
            {
#if LOG_SCULPTING_PERFORMANCE
                base::PerformanceTimer timer("\tSculpting: dilate mask");
#endif
                
                mask.dilate(mask_dilation);
                LOGMSG("dilated mask");
            }
            progress.throwIfCancelled();

            std::vector<PSLG<Scalar,Int> > pslgs;
            {
#if LOG_SCULPTING_PERFORMANCE
                base::PerformanceTimer timer("\tSculpting: mask to PSLG");
#endif
                
                // Create a PSLG for each connected component in the mask.
                bitmask_to_component_outline_pslgs<Scalar,Int>(
                                                               pslgs,
                                                               mask,
                                                               -1,
                                                               outline_accuracy,
                                                               options.image_origin(),
                                                               options.is_add_holes()
                                                               );
                LOGMSG("found " << pslgs.size() << " components");
            }
            progress.throwIfCancelled();

            return pslgs;
        }

        /**
         * Calculate the triangle size for a single puppet based on complexity and constraints
         */
        float calculatePuppetTriangleSize(const PSLGResult& pslgResult,
                                          float sqrtOfMinTriangleArea,
                                          float sqrtOfMaxTriangleArea,
                                          float minComplexity,
                                          double maxComplexity,
                                          double maxTrianglesPerPuppet,
                                          double outlineAccuracy)
        {
            // Calculate the complexity for the puppet
            const float complexityEstimate = pslgResult.perimeterEstimate / sqrt(pslgResult.areaEstimate);

            // Normalize complexity estimate from [minComplexity, maxComplexity] to [0,1] range and clamp
            const float normalizedComplexity = std::clamp((complexityEstimate - minComplexity) / float(maxComplexity - minComplexity), 0.0f, 1.0f);

            // Inverse mapping: high complexity → small triangle size
            const float inverseFactor = 1.0f - normalizedComplexity;
            
            // Lerp between min and max triangle sizes
            const float lerpedSqrtTriangleArea = numerics::lerp(sqrtOfMinTriangleArea, sqrtOfMaxTriangleArea, inverseFactor);

            // Enforce max triangles per puppet by calculating minimum triangle area for this puppet
            const float sqrtOfMinTriangleAreaForPuppet = sqrt(pslgResult.areaEstimate / maxTrianglesPerPuppet);
            const float sqrtTriangleArea = std::max(lerpedSqrtTriangleArea, sqrtOfMinTriangleAreaForPuppet);

            // Make sure the triangle size is at least the outline accuracy, since there's no point in having triangles
            // smaller than the detail in the outline
            const float finalSqrtTriangleArea = std::max(sqrtTriangleArea, float(outlineAccuracy));

            return finalSqrtTriangleArea;
        }

        /**
         * Helper function to create PSLGs using AutoZone algorithm and calculate triangle sizes for each puppet
         */
        void createPSLGsWithAutoZone(composition::EffectFullRasterSource& source,
                                     int sourceLevel,
                                     double outlineAccuracy,
                                     bool treatAlphaAsInverted,
                                     const model::PuppetSettings& puppetSettings,
                                     std::vector<PSLG<Scalar,Int>>& pslgs,
                                     std::vector<float>& triangleSizes)
        {
            USING_OPTIMTOOLS_MESH_TYPES(typename,Scalar,2);

            // First, create the PSLGs using AutoZone
            const std::vector<PSLGResult> pslgResults =
                PuppetPSLGAutoZoneAdapter::createPSLGs(source, sourceLevel, outlineAccuracy, treatAlphaAsInverted);

            // Add PSLGs from the results to the output vector
            pslgs.clear();
            pslgs.reserve(pslgResults.size());
            for (const auto& result : pslgResults)
            {
                pslgs.push_back(result.pslg);
            }

            // Now, we need to calculate the triangle sizes for each puppet based on a plethora
            // of constants as well as a complexity estimate for each puppet.

            // Sum up area estimates from all zones
            int32_t totalAreaEstimate = 0;
            for (const auto& result : pslgResults)
            {
                totalAreaEstimate += result.areaEstimate;
            }
            
            // Calculate the min and max triangle area for the entire image based on the
            // total area and the min and max triangle counts
            const float sqrtOfMinTriangleArea = sqrt(totalAreaEstimate / puppetSettings.maxTriangles);
            const float sqrtOfMaxTriangleArea = sqrt(totalAreaEstimate / puppetSettings.minTriangles);

            // Set the minimum complexity value that is geometrically possible, which is the complexity of a circle.
            // Complexity is deemed to be perimeter / sqrt(area)
            const float minComplexity = 3.5449f; // 2 * sqrt(PI)

            // Determine the triangle sizes for each puppet. To do so, we calculate the complexity of the puppet,
            // then lerp between the min triangle size and max triangle size based on the complexity. More
            // complex puppets require smaller triangle sizes.
            triangleSizes.clear();
            triangleSizes.reserve(pslgResults.size());
            for (size_t i = 0; i < pslgResults.size(); ++i)
            {
                const float finalSqrtTriangleArea = calculatePuppetTriangleSize(pslgResults[i],
                                                                                sqrtOfMinTriangleArea,
                                                                                sqrtOfMaxTriangleArea,
                                                                                minComplexity,
                                                                                puppetSettings.maxComplexity,
                                                                                puppetSettings.maxTrianglesPerPuppet,
                                                                                outlineAccuracy);
                triangleSizes.push_back(finalSqrtTriangleArea);
            }
        }

        std::vector<shared_ptr<PuppetT>> make_puppets(composition::EffectFullRasterSource& source,
                                                      geom::Vector2f sourceOrigin,
                                                      bool treatAlphaAsInverted,
                                                      const model::PuppetSettings& puppetSettings,
                                                      int32_t retry,
                                                      const base::Progress& progress)
        {
            // check progress, and throw if cancelled.
            progress.throwIfCancelled();

            // Determine the level to retrieve from the 'source'. Since the source can be large,
            // we need to retrieve at a level that can fit in one texture.
            float scale = 1;
            int sourceLevel = 0;
            const geom::Box2f& sourceBounds = source.sourceBounds();
            const geom::Vector2f size0(sourceBounds.xsize(), sourceBounds.ysize());
            geom::Vector2i isize(std::ceil(size0.x), std::ceil(size0.y));
            uint32_t sourceArea = isize.x * isize.y;
            constexpr uint32_t maxArea = 4096*4096;  // Downsample to 4k and below
            constexpr uint32_t maxDim = 16384; // Textures cannot be allocated with a max dimension above this.
            while (sourceArea > maxArea || std::max(isize.x, isize.y) > maxDim)
            {
                scale *= 0.5;
                sourceLevel += 1;
                
                geom::Vector2f size = size0 * scale;
                isize = geom::Vector2i(std::ceil(size.x), std::ceil(size.y));
                sourceArea = isize.x * isize.y;
            }

            // Add additional downsampling for each attempt that has failed
            sourceLevel += retry;
            sourceLevel = std::min(sourceLevel, source.numLevels() - 1); // clamp to max level

            // Choose between legacy and AutoZone based on puppet settings
            std::vector<PSLG<Scalar,Int>> pslgs;
            double outline_accuracy = default_outline_accuracy(isize.x, isize.y);

            // Vector of triangle sizes, one per puppet
            std::vector<float> triangleSizes;

            if (puppetSettings.useAutoZone)
            {
                createPSLGsWithAutoZone(source, sourceLevel, outline_accuracy, treatAlphaAsInverted,
                                        puppetSettings, pslgs, triangleSizes);
            }
            else
            {
                // Currently, this code path always gets the full source size. It does not make
                // any attempt to get data at a smaller level
                
                uint32_t xsize = 0;
                uint32_t ysize = 0;
                std::vector<uint8_t> pixels;
                {
                    // Limit scope of the GPU target & VirtualTexture, as only the pixel data
                    // is required for puppet creation (which can be time-consuming).
                    renderer::GPUTargetFactory::LocalTarget target(renderer::GPUTargetFactory::instance());
                    std::shared_ptr<renderer::ConstVirtualTexture> vtexture = source.texture(*target);

                    xsize = vtexture->width();
                    ysize = vtexture->height();
                    pixels.resize(xsize*ysize*4);
                    {
                        renderer::VirtualTexture::Read texture(*target, *vtexture);
                        texture->getPixels((*target).renderContext(), 0, 0, xsize, ysize, pixels.data());
                    }
                }
                
                // Make puppets
                PuppetFromImageOptions options;
                options.image_origin()[0] = sourceOrigin.x;
                options.image_origin()[1] = sourceOrigin.y;
                PuppetImage image(xsize, ysize, pixels.data());
                
                pslgs = create_image_puppet_pslgs(&image, options, progress, treatAlphaAsInverted);
                
                // For legacy path, use the same triangle size for all puppets
                float sqrtOfMaxTriangleArea = 18 * outline_accuracy;
                triangleSizes.clear();
                triangleSizes.resize(pslgs.size(), sqrtOfMaxTriangleArea);
            }

            // the creation of the PSLGs above could be time-consuming.
            // check progress again, and throw if cancelled.
            progress.throwIfCancelled();

#if LOG_SCULPTING_PERFORMANCE
            printf("Sculpting: objects found: %zu\n", pslgs.size());
#endif
            
            std::vector<shared_ptr<PuppetT>> puppets;
            for (size_t i = 0; i < pslgs.size(); ++i)
            {
                const auto& pslg = pslgs[i];

#if LOG_SCULPTING_PERFORMANCE
                // Extract PSLG properties: number of outlines, number of holes, total number of points
                size_t num_outlines = pslg.num_outlines();
                size_t num_holes = pslg.num_outlines(true);  // true means count holes
                size_t num_points = pslg.num_points();
               printf("Sculpting: puppet #%zu, outlines: %zu, holes: %zu, points: %zu\n",
                      i, num_outlines, num_holes, num_points);
#endif
                
                // Make a new puppet
                PuppetFigureT figure(pslg);
                auto puppet = make_shared<SculptingPuppet<Scalar>>(figure);

                // Set the puppet parameters
                // Note: Auto-pin settings are NOT set here; they are managed by
                // PuppetStroker and passed to puppets at drag time via add_handle()
                puppet->set_triangle_size(triangleSizes[i]);
                puppet->set_bird_w(puppetSettings.w);
                puppet->set_bird_s(puppetSettings.s);

                // Initialize the warper so the puppet is ready to use
                // TY_TODO: Probably worth exposing an init_warper() method
                puppet->iterate_warper();
                puppets.push_back(puppet);
            }

            return puppets;
        }

        std::vector<shared_ptr<PuppetT>> make_puppets(composition::EffectFullRasterSource& source,
                                                      geom::Vector2f sourceOrigin,
                                                      bool treatAlphaAsInverted,
                                                      const model::PuppetSettings& puppetSettings,
                                                      const base::Progress& progress)
        {
            // After producing the PSLG, it is possible that the triangulation logic throws an
            // internal exception upon encountering 'bad' triangles. While fixing these cases is
            // ideal, we also add a mechanism for retrying the puppet creation. With each retry,
            // we increase the downsampling of the source image, which can help remove incredibly
            // fine details that cause triangulation issues.
            constexpr int32_t kMaxAttempts = 3;
            for (int32_t retry = 0; retry < kMaxAttempts; ++retry)
            {
                try
                {
                    return make_puppets(source, sourceOrigin, treatAlphaAsInverted, puppetSettings, retry, progress);
                } 
                catch (const std::exception&)
                {
                }
            }

            // If we reach here, all attempts failed, so re-throw the error
            throw std::runtime_error("Failed to create puppets after multiple attempts");
        }

        /** Prints out postscript for the puppet mesh. This can be pasted into
            a text file and then rendered.
         */
        void print_puppet(PuppetT& puppet)
        {
            USING_OPTIMTOOLS_MESH_TYPES(typename,Scalar,2);

            printf("%% Puppet shape\n");
            puppet.figure_shape().loop_segments([&](const Point& p1, const Int& id1, const Point& p2, const Int& id2)
            {
                if (id1 != id2 || id1 < 0)
                {
                    //glColor3d(0.0, 0.0, 0.0);
                }
                else
                {
                    //glColor3d(1.0, 0.0, 0.0);
                }
                printf("newpath %g %g moveto %g %g lineto stroke\n", p1.x(), p1.y(), p2.x(), p2.y());
                //glVertex3d(p1.x(), p1.y(), 0.0);
                //glVertex3d(p2.x(), p2.y(), 0.0);
            });
            
            if (puppet.base_mesh().is_init())
            {
                printf("%% Puppet base mesh\n");
                {
                    const PointVector& points = puppet.base_mesh().points();
                    const TriangleVector& triangles = puppet.base_mesh().triangles();
                    for(Int triangle_ind = 0; triangle_ind < triangles.size(); ++triangle_ind)
                    {
                        const Triangle& tri = triangles[triangle_ind];
                        const Point& p1 = points[tri[0]];
                        const Point& p2 = points[tri[1]];
                        const Point& p3 = points[tri[2]];
                        printf("newpath %g %g moveto %g %g lineto stroke\n", p1.x(), p1.y(), p2.x(), p2.y());
                        printf("newpath %g %g moveto %g %g lineto stroke\n", p2.x(), p2.y(), p3.x(), p3.y());
                        printf("newpath %g %g moveto %g %g lineto stroke\n", p3.x(), p3.y(), p1.x(), p1.y());
                    }
                }

                printf("%% Puppet deformed mesh\n");
                {
                    const PointVector& points = puppet.deformed_mesh().points();
                    const TriangleVector& triangles = puppet.deformed_mesh().triangles();
                    printf("0 1 0 setrgbcolor\n");
                    for(Int triangle_ind = 0; triangle_ind < triangles.size(); ++triangle_ind)
                    {
                        const Triangle& tri = triangles[triangle_ind];
                        const Point& p1 = points[tri[0]];
                        const Point& p2 = points[tri[1]];
                        const Point& p3 = points[tri[2]];
                        printf("newpath %g %g moveto %g %g lineto stroke\n", p1.x(), p1.y(), p2.x(), p2.y());
                        printf("newpath %g %g moveto %g %g lineto stroke\n", p2.x(), p2.y(), p3.x(), p3.y());
                        printf("newpath %g %g moveto %g %g lineto stroke\n", p3.x(), p3.y(), p1.x(), p1.y());
                    }
                }
            }
        }


        //--------------------------------------------------------------------------------
        // PuppetEffect
        //--------------------------------------------------------------------------------

        bool PuppetEffect::beginSamples(const PuppetStrokeSample& sample, Scalar hitRadius)
        {
            // Indicate that a change is about to be made
            dispatchChanging(geom::Box2f::infinite(), true);

            // Delegate to PuppetStroker for background thread processing
            bool beginSamples = _puppetStroker->beginSamples(sample, hitRadius);

            _currentState = nullptr;

            return beginSamples;
        }
    
        void PuppetEffect::addSample(const PuppetStrokeSample& sample)
        {
            // Indicate that a change is about to be made
            dispatchChanging(geom::Box2f::infinite(), true);

            // Delegate puppet deformation to background thread
            _puppetStroker->addSample(sample);

            _currentState = nullptr;
        }
    
        void PuppetEffect::endSamples()
        {
            // Indicate that a change is about to be made
            // This is required because PuppetStroker::endSamples() is a synchronous
            // operation that waits for any outstanding samples to be processed, which
            // could lead to the effect being modified.
            dispatchChanging(geom::Box2f::infinite(), true);

            // Wrap up any remaining samples and ensure the puppet is fully updated
            _puppetStroker->endSamples();
        }
    
        void PuppetEffect::setShowMesh(bool showMesh)
        {
            _puppetSettings.showMesh = showMesh;
        }
        
        void PuppetEffect::setAutoPinEnabled(bool enabled)
        {
            _puppetSettings.autoPinEnabled = enabled;
            // Delegate directly to the stroker which is the single source of truth
            // for auto-pin settings within sketch-extension
            if (_puppetStroker)
            {
                _puppetStroker->setAutoPinEnabled(enabled);
            }
        }
        
        void PuppetEffect::setAutoPinFraction(double fraction)
        {
            _puppetSettings.autoPinFurthestFraction = fraction;
            // Delegate directly to the stroker which is the single source of truth
            // for auto-pin settings within sketch-extension
            if (_puppetStroker)
            {
                _puppetStroker->setAutoPinFraction(fraction);
            }
        }
    
        /** Restore to the state in the memento and set the memento
            state to the state prior to the restore.
         */
        void PuppetEffect::swapState(composition::ElementMemento& m_)
        {
            Memento& m = static_cast<Memento&>(m_);

            if (m._data == _currentState)
                return;
            
            // save state if needed
            if (!_currentState)
            {
                saveState();
            }

            // notify that state is changing
            dispatchChanging(geom::Box2f::infinite(), false);

            // restore puppet states from the memento
            if (m._data && m._data->puppetStrokerState)
            {
                // Apply the puppet stroker state (includes puppet ordering and internal mesh states)
                _puppetStroker->setPuppetStrokerState(*m._data->puppetStrokerState);
            }

            // swap states since current is now the restored state
            std::swap(_currentState, m._data);
        }

        const PuppetT& PuppetEffect::selectedPuppet() const
        {
            return _puppetStroker->selectedPuppet();
        }

        void PuppetEffect::initializePuppets(composition::EffectFullRasterSource& image,
                                             const geom::Vector2f& imageOrigin,
                                             bool treatAlphaAsInverted,
                                             const base::Progress& progress)
        {
            std::vector<shared_ptr<PuppetT>> puppets = make_puppets(image, imageOrigin, treatAlphaAsInverted,
                                                                    _puppetSettings, progress);
            _puppetStroker->initialize(puppets);
            _puppetStroker->setAutoPinEnabled(_puppetSettings.autoPinEnabled);
            _puppetStroker->setAutoPinFraction(_puppetSettings.autoPinFurthestFraction);

            // Configure the iteration strategy with settings from PuppetSettings
            PuppetDynamicIterationStrategy& strategy = _puppetStroker->iterationStrategy();
            strategy.setMaxTimeSpacing(_puppetSettings.maxTimeSpacing);
            strategy.setMinIterations(static_cast<int64_t>(_puppetSettings.minIterations));
            strategy.setMaxIterations(_puppetSettings.maxIterations);
            strategy.setCatchupTime(_puppetSettings.catchupTime);
        }
    
        void PuppetEffect::deInitializePuppets()
        {
            _puppetStroker->releasePuppets();
            _currentState = nullptr;
        }

        void PuppetEffect::setPuppet(std::shared_ptr<PuppetT> puppet)
        {
            // Create a vector with the single puppet and initialize PuppetStroker with it
            std::vector<std::shared_ptr<PuppetT>> puppets;
            puppets.push_back(std::move(puppet));
            _puppetStroker->initialize(puppets);
            _currentState = nullptr;
            
            // notify that state is changing
            dispatchChanging(geom::Box2f::infinite(), false);
        }

        /** Get a memento of the current state.
         */
        std::unique_ptr<composition::ElementMemento> PuppetEffect::getState()
        {
            // save state if needed
            if (!_currentState)
            {
                saveState();
            }

            // make a memento with the state
            auto m = std::make_unique<Memento>();
            m->_data = _currentState;
            return m;
        }
        
        /** Save the current state of all puppets.
         * Resets the current state pointer and stashes the order of the puppets
         * as well as the state required to reset them.
         */
        void PuppetEffect::saveState()
        {
            // Create a new state data object
            _currentState = std::make_shared<StateData>();
            
            // Get the puppet stroker state (includes puppet ordering and internal mesh states)
            _currentState->puppetStrokerState = _puppetStroker->puppetStrokerState();
        }
        
        void PuppetEffect::dispatchChanging(const geom::Box2f& damage, bool recorded)
        {
            if (recorded)
            {
                willChange(MementoProvider(*this));
            }
            if (!damage.empty())
                dispatchEvent(composition::EffectChanging(*this, damage));
        }
        
        void PuppetEffect::invalidate()
        {
            dispatchChanging(geom::Box2f::infinite(), /* recorded */ false);
        }
    }
}
