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

#include <unordered_set>

#if __APPLE__
#include "composition/impl/apple/AppleCGDrawingLayerSurface.h"
#elif WIN32
#include "composition/impl/win/D2DDrawingLayerSurface.h"
#elif ASK_WEB
#include "composition/impl/web/SkiaDrawingLayerSurface.h"
#endif

#include "base/threads/Threads"
#include "composition/Composition"
#include "composition/effects/EffectFullRasterSource.h"
#include "graphics/GraphicDuplicator.h"

#include "PuppetEffectVector.h"
#include "PuppetVectorSkinner.h"

namespace extensions
{
    namespace puppet
    {
        USING_OPTIMTOOLS_MESH_TYPES(typename,Scalar,2);
        using PuppetMeshRef = std::shared_ptr<optimtools::PuppetMesh<Scalar>>;

    //--------------------------------------------------------------------------------
        // PuppetEffectVector
        //--------------------------------------------------------------------------------

        class ConcretePuppetEffectVector : public PuppetEffectVector
        {
            // Renderer content subclass that draws a given display list context to the
            // provided context
            class Content : public renderer::Content
            {
                renderer::DisplayListContext _dl;
            public:
                explicit Content()
                {
                }
                
                renderer::DisplayListContext& dlContext()
                {
                    return _dl;
                }
                
                // Simply play the display list into the provided context
                size_t draw(renderer::Context& context, const geom::Box2f& bounds, size_t begin,
                            const base::AnyValue& fbegin, size_t end, bool& cancel) const override
                {
                    _dl.play(context);
                    return end;
                }
            };
            
            struct PuppetMeshUpdates
            {
                // mutex protecting access to member variables
                mutable std::mutex _meshMutex;
                
                // the deformed mesh snapshots taken during sampling
                std::vector<PuppetMeshRef> _puppetMeshes;
                
                // cached bounds of output vector contents
                geom::Box2f _bounds;
                
                // Content instance that holds the display list (drawing commands)
                // for the output vectors
                std::shared_ptr<Content> _content;
                
                // tracks whether a call to applyPuppetsInner() is already in the queue
                bool _isApplyTaskQueued = false;
            };
            
            // the input node and composition
            composition::GenericNode& _inputNode;
            composition::Composition& _composition;
            
            // the output vector contents
            std::shared_ptr<graphics::Group> _group;
            
            // data shared between main thread and apply puppets queue
            std::shared_ptr<PuppetMeshUpdates> _puppetMeshUpdates;
            
            // the main gateway to the vector sculpting logic
            std::shared_ptr<PuppetVectorSkinner> _puppetSkinner;
            
            // provider returned for rendering the output vectors
            std::shared_ptr<composition::TiledContentLayer> _outputProvider;
            
            // queue where puppets are applied to the vectors
            std::shared_ptr<volatile base::JobQueue> _applyPuppetsQueue;

        public:
            using super = PuppetEffectVector;
            
            ConcretePuppetEffectVector(composition::ElementCollection* collection,
                                       std::shared_ptr<volatile base::JobDispatcher> jobDispatcher,
                                       composition::GenericNode& vectorNode,
                                       composition::Composition& composition) :
                PuppetEffectVector(collection, jobDispatcher),
                _inputNode(vectorNode),
                _composition(composition)
            {
                if (!_inputNode.contents().is<composition::VectorNodeContents>())
                    throw std::invalid_argument("Invalid node type passed to PuppetEffectVector");
                
                // Create the TiledContentLayer instance that will be used to render the vector output
                const composition::Universe& universe = composition::Universe::instance();
                composition::TiledContentLayer::Universe tiledContentLayerUniverse(universe.gpuFactory(),
                                                                                   universe.asyncQueue(),
                                                                                   universe.workScheduler());
                auto createSurfaceFn = [](float width, float height, float scale)
                {
                    #if __APPLE__
                        using GridTileSurface = composition::AppleCGDrawingLayerSurface;
                    #elif WIN32
                        using GridTileSurface = composition::D2DDrawingLayerSurface;
                    #elif ASK_WEB
                        using GridTileSurface = SkiaDrawingLayerSurface;
                    #endif
                    
                    auto result = std::make_unique<GridTileSurface>(width, height, scale);
                    return result;
                };
                _outputProvider = composition::TiledContentLayer::create(tiledContentLayerUniverse,
                                                                         512, /*tileSize0*/
                                                                         1.0, /*scale0*/
                                                                         createSurfaceFn,
                                                                         -8,  /*minLevel*/
                                                                         8    /*maxLevel*/);
            }
            
            virtual ~ConcretePuppetEffectVector()
            {
            }

            /** The target content of the node to which the effect is supplied. Only works on content
                since masks require raster deformation.
             */
            Target target() const override
            {
                return Target::content;
            }
            
            /** The output type that the effect wishes to render.
             */
            OutputType outputType() const override
            {
                return OutputType::vector;
            }

            /** Returns the bounds of the rendered result of the effect. For example
                when the opacity changes on the node where the effect is applied, a
                change event must be dispatched with the damage bounds.
             */
            geom::Box2f damageBounds() const override
            {
                if (_puppetMeshUpdates)
                {
                    std::lock_guard<std::mutex> lock(_puppetMeshUpdates->_meshMutex);
                    return _puppetMeshUpdates->_bounds;
                }
                else
                {
                    return geom::Box2f();
                }
            }
            
            /** Flush any pending work required so that the image() will be up to
                date.
             */
            void update(const base::JobGroup::Member& group) override
            {
                if (_puppetMeshUpdates)
                {
                    std::lock_guard<std::mutex> lock(_puppetMeshUpdates->_meshMutex);
                    if (_puppetMeshUpdates->_content)
                    {
                        // Update the content instance on the TiledContentLayer to have the latest
                        // available vectors
                        composition::TiledContentLayer::Access output(*_outputProvider);
                        output.setContent(_puppetMeshUpdates->_content);
                        output.setActive(true);
                        output.setNeedsUpdate();
                        
                        // Clear the member variable so that we don't set the output provider
                        // as needing to be updated until the apply puppets queue provides us
                        // with new content.
                        _puppetMeshUpdates->_content = nullptr;
                    }
                }
            }
            
            /** Effect does not support image output, so just return blank image.
             */
            shapes::Image image() const override
            {
                return shapes::Image();
            }
            
            /** Return output vectors in TiledContentLayer.
             */
            std::shared_ptr<composition::TiledContentLayer> acquireOutput() const override
            {
                return _outputProvider;
            }
            
            /** Notifies the effect that its input will change. Should return the
                bounds that needs re-rendering.
             */
            geom::Box2f sourceChanging(const geom::Box2f& r) override
            {
                // due to warping a change to the source could affect any part of
                // the result
                return geom::Box2f::infinite();
            }
            
            /** Override endSamples() to ensure all work on apply puppets queue has completed.
             */
            void endSamples() override
            {
                // Call base class functionality
                super::endSamples();
                
                // Wait for all jobs to finish on our apply puppets queue
                _applyPuppetsQueue->wait_finished();
            }
            
            void setPuppet(std::shared_ptr<PuppetT> puppet) override
            {
                // Wait for all jobs to finish on our apply puppets queue
                _applyPuppetsQueue->wait_finished();

                {
                    std::lock_guard<std::mutex> lock(_puppetMeshUpdates->_meshMutex);
                    
                    // Call base class functionality to assign the puppet
                    super::setPuppet(puppet);
                    
                    // Duplicate the contents of srcGroup into destGroup
                    _group = graphics::Group::create(nullptr);
                    graphics::GraphicDuplicator::duplicateChildren(inputRootGroup(), *_group);
                    
                    // Initialize the puppet skinner, which binds the vectors to the base mesh
                    initializePuppetSkinner(base::Progress());
                }
                
                // Run the puppet logic, which will now use the deformed mesh.
                // Wait for the jobs to finish.
                applyPuppets();
                _applyPuppetsQueue->wait_finished();
            }
            
            void initialize(const model::PuppetSettings& puppetSettings,
                            const base::Progress& progress) override
            {
                if (_applyPuppetsQueue)
                {
                    // Wait for all jobs to finish on our apply puppets queue
                    _applyPuppetsQueue->wait_finished();
                }
                else
                {
                    // Grab the static instance of the queue
                    _applyPuppetsQueue = applyPuppetsQueue();
                }

                _puppetMeshUpdates = std::make_shared<PuppetMeshUpdates>();
                {
                    std::lock_guard<std::mutex> lock(_puppetMeshUpdates->_meshMutex);
                    
                    // Capture the puppet settings
                    _puppetSettings = puppetSettings;
                    
                    // Initialize the source raster image
                    std::shared_ptr<renderer::VirtualTextureArrayAllocator> allocator = _composition.factories().allocator();

                    // Get size of contents
                    _puppetMeshUpdates->_bounds = graphics::AllGraphicsTightBounds::calculate(inputRootGroup());
                    if (_puppetMeshUpdates->_bounds.empty())
                        return;
                    
                    // Outset the bounds and ensure it falls on integer boundaries with even dimensions.
                    // This isn't strictly necessary, but since we can't have a fractional number of pixels in an image,
                    // it feels a little more predictable to specify the integer bounds explicitly here, rather than
                    // rely on the underlying system rendering the vectors on an integer-sized grid. Additionally, the
                    // final translation of the puppet outlines to the bounds' origin would result in an integer translation,
                    // which slightly simplifies the transformation to the outline results.
                    // Having an even number for the dimensions also feels cleaner so that at least the first
                    // downsample is always divisible by two.
                    geom::Box2f rasterSourceBounds = _puppetMeshUpdates->_bounds;
                    rasterSourceBounds.inset(-5, -5);
                    rasterSourceBounds.integral();
                    if (int64_t(rasterSourceBounds.xsize()) % 2 != 0)
                        rasterSourceBounds.xmax += 1;
                    if (int64_t(rasterSourceBounds.ysize()) % 2 != 0)
                        rasterSourceBounds.ymax += 1;

                    // create the source for the surrogate from the layer contents
                    composition::EffectFullRasterSource::Params params;
                    params.bounds(rasterSourceBounds);
                    params.allocator(std::move(allocator));
                    params.target(composition::Effect::Target::content);
                    params.alphaFormat(renderer::AlphaFormat::unmul);
                    params.areaThreshold(1);
                    auto source = composition::EffectFullRasterSource::create(_inputNode, params);
                    _composition.flushOutput().check();
                    
                    // Initialize the puppets
                    initializePuppets(*source,
                                      geom::Vector2f(rasterSourceBounds.xmin, rasterSourceBounds.ymin),
                                      false /*treatAlphaAsInverted*/,
                                      progress);
                    
                    // Duplicate the contents of srcGroup into destGroup
                    _group = graphics::Group::create(nullptr);
                    graphics::GraphicDuplicator::duplicateChildren(inputRootGroup(), *_group);
                    
                    // Create and initialize the puppet skinner
                    _puppetSkinner = PuppetVectorSkinner::create();
                    initializePuppetSkinner(progress);
                    
                    // Set the lambda callback when puppets are updated by the stroker
                    _puppetStroker->setPuppetsUpdatedCallback([this]()
                    {
                        this->applyPuppets();
                    });
                    
                    // Populate the display list with the initial copy of the input group
                    // Create new Content instance and draw to its display list
                    _puppetMeshUpdates->_content = std::make_shared<Content>();
                    auto& dlContext = _puppetMeshUpdates->_content->dlContext();
                    graphics::VisibleGraphicsRenderer vgr(dlContext, _group.get(), 0);
                    vgr.draw(*_group);
                }
            }
            
            void deInitialize() override
            {
                // Wait for all jobs to finish on our apply puppets queue
                _applyPuppetsQueue->wait_finished();

                {
                    std::lock_guard<std::mutex> lock(_puppetMeshUpdates->_meshMutex);

                    // Release output group
                    _group = nullptr;
                    
                    // Release puppet mesh data
                    _puppetMeshUpdates->_puppetMeshes.clear();

                    // Release the Content and display list
                    _puppetMeshUpdates->_content = nullptr;

                    // Release the puppet skinner
                    _puppetSkinner = nullptr;
                    
                    // Release the apply puppets queue
                    _applyPuppetsQueue = nullptr;
                    
                    // Deinitialize the puppets
                    deInitializePuppets();
                    
                    // Create new Content instance with empty display list and set it on output provider
                    composition::TiledContentLayer::Access output(*_outputProvider);
                    output.setContent(std::make_shared<Content>());
                }
                
                // Release the puppet mesh updates
                _puppetMeshUpdates = nullptr;
            }
            
            std::shared_ptr<const graphics::Group> outputVectors() const override
            {
                // Wait for all jobs to finish on our apply puppets queue
                _applyPuppetsQueue->wait_finished();

                return _group;
            }

        private:
            static std::shared_ptr<volatile base::JobQueue> applyPuppetsQueue()
            {
                // Share the same queue among all instances
                static std::weak_ptr<volatile base::JobQueue> sApplyPuppetsQueue;
                auto result = sApplyPuppetsQueue.lock();
                if (!result)
                {
                    result = base::JobQueue::createAsyncQueue("PuppetEffectVector apply queue",
                                                              base::Thread::QOS::normal);
                    sApplyPuppetsQueue = result;
                }
                return result;
            }
            
            const graphics::Group& inputRootGroup()
            {
                return _inputNode.contents().as<composition::VectorNodeContents>().rootGroup();
            }
            
            // This method assumes the mesh mutex is already acquired and no jobs are running on the applyPuppetsQueue
            void initializePuppetSkinner(const base::Progress& progress)
            {
                // Capture the current state of the puppet meshes
                std::vector<PuppetMeshRef> puppetMeshes;
                const auto& puppets = _puppetStroker->puppets();
                for (int i=0; i< puppets.size(); i++)
                {
                    // Copy base mesh from puppet
                    std::shared_ptr<PuppetT> puppet = puppets[i];
                    puppetMeshes.push_back(std::make_shared<optimtools::PuppetMesh<Scalar>>(puppet->base_mesh()));
                }
                
                // Initialize the puppet skinner
                _puppetSkinner->initialize(_group, puppetMeshes, progress);
            }
            
            // Called downstream of PuppetStroker when the puppets are updated
            void applyPuppets()
            {
                std::lock_guard<std::mutex> lock(_puppetMeshUpdates->_meshMutex);

                // Capture the current state of the puppet meshes
                _puppetMeshUpdates->_puppetMeshes.clear();
                const auto& puppets = _puppetStroker->puppets();
                for (int i=0; i< puppets.size(); i++)
                {
                    // Copy mesh from puppet
                    std::shared_ptr<PuppetT> puppet = puppets[i];
                    const auto& puppetMesh = puppet->deformed_mesh();
                    auto deformedPoints = std::make_shared<PointVector>(puppetMesh.points());
                    std::shared_ptr<TriangleVector> deformedTriangles = puppetMesh.triangles_ptr();
                    std::shared_ptr<IntVector> deformedPointHandles = puppetMesh.point_handles_ptr();
                    
                    _puppetMeshUpdates->_puppetMeshes.push_back(
                        std::make_shared<optimtools::PuppetMesh<Scalar>>(deformedPoints,
                                                                         deformedTriangles,
                                                                         deformedPointHandles));
                }
                
                // If we currently have nothing queued, queue up the background work
                if (!_puppetMeshUpdates->_isApplyTaskQueued)
                {
                    std::weak_ptr<ConcretePuppetEffectVector> weakThis =
                        std::static_pointer_cast<ConcretePuppetEffectVector>(shared_from_this());
                    
                    // Schedule on our apply puppets queue
                    _applyPuppetsQueue->push([weakThis,
                                              puppetSkinner = _puppetSkinner,
                                              puppetMeshUpdates = _puppetMeshUpdates]()
                    {
                        applyPuppetsInternal(weakThis, puppetSkinner, puppetMeshUpdates);
                    });
                    _puppetMeshUpdates->_isApplyTaskQueued = true;
                }
            }
            
            // Run on the '_applyPuppetsQueue'
            static void applyPuppetsInternal(std::weak_ptr<ConcretePuppetEffectVector> weakThis,
                                             std::shared_ptr<PuppetVectorSkinner> puppetSkinner,
                                             std::shared_ptr<PuppetMeshUpdates> puppetMeshUpdates)
            {
                std::vector<PuppetMeshRef> puppetMeshes;
                {
                    std::lock_guard<std::mutex> lock(puppetMeshUpdates->_meshMutex);
                    
                    // Copy all the values from the variables protected by the mutex
                    puppetMeshUpdates->_isApplyTaskQueued = false;
                    std::swap(puppetMeshUpdates->_puppetMeshes, puppetMeshes);
                }
                               
                // Run the skinner
                puppetSkinner->applyPuppets(puppetMeshes);
                
                // Create a new Content instance and draw to its display list
                auto group = puppetSkinner->outputVectors();
                auto content = std::make_shared<Content>();
                auto& dlContext = content->dlContext();
                graphics::VisibleGraphicsRenderer vgr(dlContext, group.get(), 0);
                vgr.draw(*group);
                
                // Get size of contents
                auto bounds = graphics::AllGraphicsTightBounds::calculate(*group);
                
                {
                    // Assign values to shared struct
                    std::lock_guard<std::mutex> lock(puppetMeshUpdates->_meshMutex);
                    puppetMeshUpdates->_bounds = bounds;
                    puppetMeshUpdates->_content = content;
                }
                
                // Dispatch changing event on the main queue so that the latest content is drawn
                base::JobQueue::mainQueue()->push([weakThis]()
                {
                    auto strongThis = weakThis.lock();
                    if (strongThis)
                    {
                        strongThis->dispatchEvent(
                            composition::EffectChanging(*strongThis, geom::Box2f::infinite()));
                    }
                });
            }
            
            // For vector effects, there is no concept of destination bounds, 
            // so this method always returns false
            bool hasContentOutsideDstBounds() const override
            {
                return false;
            }
        };
    
        std::shared_ptr<PuppetEffectVector>
            PuppetEffectVector::create(composition::ElementCollection* collection,
                                       std::shared_ptr<volatile base::JobDispatcher> jobDispatcher,
                                       composition::GenericNode& vectorNode,
                                       composition::Composition& composition)
        {
            return std::make_shared<ConcretePuppetEffectVector>(collection, jobDispatcher, vectorNode, composition);
        }
    }
}


