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

#include "PuppetEffectRaster.h"
#include "PuppetImage.h"

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
        USING_OPTIMTOOLS_MESH_TYPES(typename, double, 2);

        void apply_puppets(const PuppetStroker& stroker,
                           PuppetImageProvider& image, bool showMesh)
        {
            std::unique_ptr<gpu::VertexBuffer> vertices;
            std::unique_ptr<gpu::IndexBuffer> indices;

            // Use stroker's createWarpBuffers method (handles synchronization and buffer creation)
            stroker.createWarpBuffers(vertices, indices);

            // Get size from image
            geom::Vector2f size(image.width(), image.height());

            image.setWarp(std::move(vertices), std::move(indices), size, showMesh);
        }
    

        //--------------------------------------------------------------------------------
        // PuppetEffectRaster
        //--------------------------------------------------------------------------------

        class ConcretePuppetEffectRaster : public PuppetEffectRaster
        {
            // the image
            std::shared_ptr<PuppetImageProvider> _image;
            
            // Track which vertices started outside image bounds during initialization
            // Each bool corresponds to a single vertex. It is assumed that the order
            // of the vertices within the stroker never changes
            std::vector<bool> _verticesStartingOutOfBounds;
            
            // Helper method to check if a point is outside the image bounds (with threshold tolerance)
            bool isPointOutsideDstBounds(float x, float y) const
            {
                // Use a tolerance to account for a small amount of jitter on vertices
                // near the edges when using the warper.
                static constexpr float kThreshold = 0.05f;
                float imageWidth = _image->width();
                float imageHeight = _image->height();
                return (x < -kThreshold || x > imageWidth + kThreshold || 
                        y < -kThreshold || y > imageHeight + kThreshold);
            }
            
            // Helper method to build the _verticesStartingOutOfBounds vector
            void trackOutOfBoundsVertices()
            {
                // Get puppets in stable order (not reordered by stacking order)
                const std::vector<std::shared_ptr<PuppetT>>& puppets = _puppetStroker->puppets();
                
                // Initialize the vector - one bool per vertex
                _verticesStartingOutOfBounds.clear();
                
                // Check each vertex in each puppet
                for (const std::shared_ptr<PuppetT>& puppet : puppets)
                {
                    const PointVector& dstPoints = puppet->deformed_mesh().points();
                    
                    for (const Point& dstPoint : dstPoints)
                    {
                        float dstX = dstPoint.x();
                        float dstY = dstPoint.y();
                        
                        // Mark true if vertex is outside image bounds (with threshold tolerance)
                        _verticesStartingOutOfBounds.push_back(isPointOutsideDstBounds(dstX, dstY));
                    }
                }
            }
            
        public:
            using super = PuppetEffectRaster;
            
            ConcretePuppetEffectRaster(composition::ElementCollection* collection,
                                       std::shared_ptr<volatile base::JobDispatcher> jobDispatcher,
                                       std::unique_ptr<composition::EffectFullRasterSource> source,
                                       float w, float h, bool treatAlphaAsInverted) :
                PuppetEffectRaster(collection, jobDispatcher)
            {
                _image = std::make_shared<PuppetImageProvider>(std::move(source),
                                                               w, h, treatAlphaAsInverted);
            }
            
            virtual ~ConcretePuppetEffectRaster()
            {
            }

            /** The target content of the node to which the effect is supplied.
             */
            Target target() const override
            {
                // since the source supplies the content ask it for the target
                return _image->source().target();
            }

            /** The output type that the effect wishes to render.
             */
            OutputType outputType() const override
            {
                return OutputType::pixel;
            }
            
            /** Returns the bounds of the rendered result of the effect. For example
                when the opacity changes on the node where the effect is applied, a
                change event must be dispatched with the damage bounds.
             */
            geom::Box2f damageBounds() const override
            {
                return geom::Box2f(0, _image->width(), 0, _image->height());
            }
            
            /** Flush any pending work required so that the image() will be up to
                date.
             */
            void update(const base::JobGroup::Member& group) override
            {
                apply_puppets(*_puppetStroker, *_image, _puppetSettings.showMesh);
            }
            
            /** Returns an image that is the rendered result of the effect.
             */
            shapes::Image image() const override
            {
                return shapes::Image(_image);
            }

            /** Effect does not support vector output, so return nullptr.
             */
            std::shared_ptr<composition::TiledContentLayer> acquireOutput() const override
            {
                return nullptr;
            }
            
            /** Notifies the effect that its input will change. Should return the
                bounds that needs re-rendering.
             */
            geom::Box2f sourceChanging(const geom::Box2f& r) override
            {
                // invalidate the cached source info
                _image->source().invalidateInput();
                
                // due to warping a change to the source could affect any part of
                // the result
                return damageBounds();
            }
            
            void initialize(const model::PuppetSettings& puppetSettings,
                            const base::Progress& progress) override
            {
                _puppetSettings = puppetSettings;
                initializePuppets(_image->source(), geom::Vector2f(0,0), _image->treatAlphaAsInverted(), progress);

                // Track which vertices start outside bounds for later use in hasContentOutsideDstBounds()
                trackOutOfBoundsVertices();
                
                apply_puppets(*_puppetStroker, *_image, _puppetSettings.showMesh);
            }
            
            void deInitialize() override
            {
                // Deinitialize the puppets
                deInitializePuppets();
            }

            void createWarpBuffers(std::unique_ptr<gpu::VertexBuffer>& outVertices,
                             std::unique_ptr<gpu::IndexBuffer>& outIndices) const override
            {
                _puppetStroker->createWarpBuffers(outVertices, outIndices);
            }

            bool treatAlphaAsInverted() const override
            {
                return _image->treatAlphaAsInverted();
            }
            
            bool hasContentOutsideDstBounds() const override
            {
                // Returns whether there is currently content of the effect that is projected 
                // outside the traditional bounds of the raster node. This corresponds to having 
                // any vertex (that was originally within the layer bounds) now fall outside 
                // the layer bounds due to puppet deformation.
                
                // Get puppets in stable order (same as used in trackOutOfBoundsVertices)
                const std::vector<std::shared_ptr<PuppetT>>& puppets = _puppetStroker->puppets();
                
                // Check each vertex in each puppet using the same stable ordering
                size_t vertexIndex = 0;
                for (const std::shared_ptr<PuppetT>& puppet : puppets)
                {
                    const PointVector& dstPoints = puppet->deformed_mesh().points();
                    
                    for (const Point& dstPoint : dstPoints)
                    {
                        // Skip vertices that were originally out of bounds
                        // since we know the actual pixel content didn't start out of bounds.
                        // The dilation of the mesh can cause vertices to be projected
                        // outside the layer bounds.
                        if (vertexIndex < _verticesStartingOutOfBounds.size() &&
                            _verticesStartingOutOfBounds[vertexIndex])
                        {
                            vertexIndex++;
                            continue;
                        }

                        float dstX = dstPoint.x();
                        float dstY = dstPoint.y();

                        // Check if this vertex is now projected outside the layer bounds
                        if (isPointOutsideDstBounds(dstX, dstY))
                        {
                            return true;
                        }
                        
                        vertexIndex++;
                    }
                }
                
                return false;
            }
            
        };

        std::shared_ptr<PuppetEffectRaster>
        PuppetEffectRaster::create(composition::ElementCollection* collection,
                                   std::shared_ptr<volatile base::JobDispatcher> jobDispatcher,
                                   std::unique_ptr<composition::EffectFullRasterSource> source,
                                   float w, float h, bool treatAlphaAsInverted)
        {
            return std::make_shared<ConcretePuppetEffectRaster>(collection,
                                                                jobDispatcher,
                                                                std::move(source),
                                                                w, h, treatAlphaAsInverted);
        }
    }
}
