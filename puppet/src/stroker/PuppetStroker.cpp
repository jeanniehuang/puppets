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

#include "PuppetStroker.h"
#include "SculptingPuppet.h"

#include <algorithm>
#include <cstdint>
#include <limits>
#include "base/threads/Threads"
#include "composition/Composition"
#include "renderer/gpu/GPU"

namespace extensions
{
    namespace puppet
    {

        /**
         * Concrete implementation of PuppetStrokerState that contains the actual data.
         */
        class ConcretePuppetStrokerState : public PuppetStroker::PuppetStrokerState
        {
        public:
            // The duplicated internal states of each puppet
            std::vector<std::unique_ptr<AbstractPuppet::PuppetState>> puppetStates;

            // Duplicated indices to each puppet, primarily captured for
            // the sake of preserving the ordering. Note that the puppet state
            // vector should have the same size as this vector
            std::vector<int32_t> puppetIndices;
            
            ConcretePuppetStrokerState() = default;
            
            // Delete copy constructor and copy assignment to prevent accidental copying
            ConcretePuppetStrokerState(const ConcretePuppetStrokerState&) = delete;
            ConcretePuppetStrokerState& operator=(const ConcretePuppetStrokerState&) = delete;
        };
        
        PuppetStroker::PuppetStroker(std::shared_ptr<volatile base::JobDispatcher> jobDispatcher) :
            _jobDispatcher(std::move(jobDispatcher)),
            _iterationStrategy()
        {
        }
        
        void PuppetStroker::initialize(const std::vector<std::shared_ptr<PuppetT>>& puppets)
        {
            // Block actions from being queued on the main thread
            std::lock_guard<std::mutex> lock(_mainThreadMutex);
            // Wait for all outstanding operations to complete on the background thread
            _queuedActionsJobGroup.wait();

            _puppets = puppets;
            
            // Create the initial stacking order
            _puppetStackingOrder.resize(_puppets.size());
            for (int i=0; i< _puppets.size(); i++)
                _puppetStackingOrder[i] = i;
        }
        
        void PuppetStroker::releasePuppets()
        {
            _puppets.clear();
            _puppetStackingOrder.clear();
        }
        
        bool PuppetStroker::beginSamples(const PuppetStrokeSample& sample, Scalar hitRadius)
        {
            // Block actions from being queued on the main thread
            std::lock_guard<std::mutex> lock(_mainThreadMutex);

            // If there are no puppets, there is nothing to modify
            if (_puppets.empty())
            {
                return false;
            }

            // Set the hit radius for this stroke session
            _outsideHitRadius = hitRadius;

            // Get the starting point from the strategy, returning early if one does not exist
            _iterationStrategy.beginStroke();
            std::optional<PuppetStrokeSample> filteredSampleOpt = _iterationStrategy.sampleReceived(sample);
            if (!filteredSampleOpt.has_value())
            {
                return false;
            }
            PuppetStrokeSample filteredSample = filteredSampleOpt.value();


            // Select the puppet, returning early if we do not hit one
            bool success = this->selectPuppetAtPoint(filteredSample.position);
            if (!success)
            {
                return false;
            }

            // Create a new job group for this batch of operations
            _queuedActionsJobGroup = base::JobGroup();

            // Submit background work for puppet selection and handle addition
            base::JobGroup::Member member(_queuedActionsJobGroup);
            _jobDispatcher->push(member, [this, filteredSample]()
            {
                this->beginSamplesBackgroundAction(filteredSample.position);
                this->addSampleBackgroundAction(filteredSample);
            });
            
            // Return that we began the stroke successfully
            return true;
        }
        
        void PuppetStroker::addSample(const PuppetStrokeSample& sample)
        {
            // Block actions from being queued on the main thread
            std::lock_guard<std::mutex> lock(_mainThreadMutex);

            // Retrieve the filtered sample from the strategy
            std::optional<PuppetStrokeSample> filteredSampleOpt = _iterationStrategy.sampleReceived(sample);
            if (!filteredSampleOpt.has_value())
            {
                return;
            }
            PuppetStrokeSample filteredSample = filteredSampleOpt.value();

            // Submit work to background thread using job group
            base::JobGroup::Member member(_queuedActionsJobGroup);
            _jobDispatcher->push(member, [this, filteredSample]()
            {
                this->addSampleBackgroundAction(filteredSample);
            });
        }
        
        void PuppetStroker::endSamples()
        {
            // Cancel any outstanding work immediately
            // Since _cancelled is atomic, it can be set without holding a mutex
            _cancelled = true;

            // Block actions from being queued on the main thread
            std::lock_guard<std::mutex> lock(_mainThreadMutex);

            // Submit endSamples work to background thread using job group
            //
            // Make sure the member is destroyed before waiting on the
            // job group, otherwise, we deadlock
            {
                base::JobGroup::Member member(_queuedActionsJobGroup);
                _jobDispatcher->push(member, [this]()
                {
                    this->endSamplesBackgroundAction();
                });
            }

            // Block until all outstanding work is completed on the background thread
            _queuedActionsJobGroup.wait();
            
            // Reset cancellation flag for next stroke
            _cancelled = false;
        }
        
        void PuppetStroker::createWarpBuffers(std::unique_ptr<gpu::VertexBuffer>& outVertices,
                                              std::unique_ptr<gpu::IndexBuffer>& outIndices) const
        {
            // Block actions from being performed on the background thread
            std::lock_guard<std::mutex> backgroundLock(_backgroundThreadMutex);
            // Block actions from being queued on the main thread
            std::lock_guard<std::mutex> lock(_mainThreadMutex);

            if (_puppets.empty())
            {
                return;
            }

            // Create GPU buffers from puppet data
            createWarpBuffers(_puppets, _puppetStackingOrder, outVertices, outIndices);
        }

        void PuppetStroker::createWarpBuffers(const std::vector<std::shared_ptr<PuppetT>>& puppets,
                                              const std::vector<int32_t>& puppetStackingOrder,
                                              std::unique_ptr<gpu::VertexBuffer>& outVertices,
                                              std::unique_ptr<gpu::IndexBuffer>& outIndices) const
        {
            // First check if we will overflow uint16_t indices
            size_t totalVertexCount = 0;
            for (int32_t puppetIndex : puppetStackingOrder)
            {
                const std::shared_ptr<PuppetT>& puppet = puppets[puppetIndex];
                totalVertexCount += puppet->deformed_mesh().points().size();
            }
            
            // If we would overflow uint16_t, use vertex-only approach (no index buffer)
            // Failure to do so would cause indices to wrap around and accidentally reference
            // incorrect vertices. The vertex buffer can support that many vertices, however,
            // so we can still render correctly without indices, we will just be a memory
            // inefficient while doing so.
            if (totalVertexCount > std::numeric_limits<uint16_t>::max())
            {
                createWarpBufferWithoutIndices(puppets, puppetStackingOrder, outVertices);
                outIndices = nullptr;
            }
            else
            {
                createWarpBuffersWithIndices(puppets, puppetStackingOrder, outVertices, outIndices);
            }
        }

        void PuppetStroker::createWarpBufferWithoutIndices(const std::vector<std::shared_ptr<PuppetT>>& puppets,
                                                           const std::vector<int32_t>& puppetStackingOrder,
                                                           std::unique_ptr<gpu::VertexBuffer>& outVertices) const
        {
            using namespace optimtools;
            USING_OPTIMTOOLS_MESH_TYPES(typename,Scalar,2);
            
            renderer::GPUTargetFactory::LocalTarget target(renderer::GPUTargetFactory::instance());

            // Create standalone vertex buffer with repeated points
            std::vector<float> combinedVBuffer;

            forEachTriangleInDepthOrder(puppets, puppetStackingOrder, [&combinedVBuffer](const Triangle& tri, const PointVector& srcpts, const PointVector& dstpts, uint16_t /*vertexOffset*/)
            {
                // Add vertex data for each vertex of the triangle
                for (int j = 0; j < 3; ++j)
                {
                    const Point& dst = dstpts[tri[j]];
                    const Point& src = srcpts[tri[j]];
                    combinedVBuffer.push_back(dst.x());
                    combinedVBuffer.push_back(dst.y());
                    combinedVBuffer.push_back(src.x());
                    combinedVBuffer.push_back(src.y());
                }
            });

            outVertices = (*target).renderContext().createVertexBuffer(combinedVBuffer.data(), combinedVBuffer.size() * sizeof(float));
        }

        void PuppetStroker::createWarpBuffersWithIndices(const std::vector<std::shared_ptr<PuppetT>>& puppets,
                                                         const std::vector<int32_t>& puppetStackingOrder,
                                                         std::unique_ptr<gpu::VertexBuffer>& outVertices,
                                                         std::unique_ptr<gpu::IndexBuffer>& outIndices) const
        {
            using namespace optimtools;
            USING_OPTIMTOOLS_MESH_TYPES(typename,Scalar,2);
            
            renderer::GPUTargetFactory::LocalTarget target(renderer::GPUTargetFactory::instance());

            // Use indexed approach
            std::vector<float> combinedVBuffer;
            std::vector<uint16_t> combinedIBuffer;
            
            // First pass: Add all vertices for all puppets
            for (int32_t puppetIndex : puppetStackingOrder)
            {
                const std::shared_ptr<PuppetT>& puppet = puppets[puppetIndex];
                const PointVector& srcpts = puppet->initial_points();
                const PointVector& dstpts = puppet->deformed_mesh().points();

                // Add vertices for this puppet
                for (size_t i = 0; i < dstpts.size(); ++i)
                {
                    // dst x,y; src u,v
                    const Point& dst = dstpts[i];
                    const Point& src = srcpts[i];
                    combinedVBuffer.push_back(dst.x());
                    combinedVBuffer.push_back(dst.y());
                    combinedVBuffer.push_back(src.x());
                    combinedVBuffer.push_back(src.y());
                }
            }
            
            // Second pass: Add triangle indices using forEachTriangleInDepthOrder helper
            forEachTriangleInDepthOrder(puppets, puppetStackingOrder, [&combinedIBuffer](const Triangle& curr_triangle, const PointVector& /*srcpts*/, const PointVector& /*dstpts*/, uint16_t vertexOffset)
            {
                uint16_t idx0 = vertexOffset + curr_triangle[0];
                uint16_t idx1 = vertexOffset + curr_triangle[1];
                uint16_t idx2 = vertexOffset + curr_triangle[2];
                
                combinedIBuffer.push_back(idx0);
                combinedIBuffer.push_back(idx1);
                combinedIBuffer.push_back(idx2);
            });

            outVertices = (*target).renderContext().createVertexBuffer(combinedVBuffer.data(), combinedVBuffer.size() * sizeof(float));
            outIndices = (*target).renderContext().createIndexBuffer(combinedIBuffer.data(), combinedIBuffer.size());
        }

        void PuppetStroker::forEachTriangleInDepthOrder(const std::vector<std::shared_ptr<PuppetT>>& puppets,
                                                        const std::vector<int32_t>& puppetStackingOrder,
                                                        std::function<void(const Triangle&, const PointVector&, const PointVector&, uint16_t)> callback) const
        {
            using namespace optimtools;
            USING_OPTIMTOOLS_MESH_TYPES(typename,Scalar,2);
            
            uint16_t vertexOffset = 0;
            
            for (int32_t puppetIndex : puppetStackingOrder)
            {
                const std::shared_ptr<PuppetT>& puppet = puppets[puppetIndex];
                
                const TriangleVector& triangles = puppet->base_mesh().triangles();
                const PointVector& srcpts = puppet->initial_points();
                const PointVector& dstpts = puppet->deformed_mesh().points();

                // Get depth-sorted triangles for this puppet
                std::vector<int64_t> depthOrder = puppet->calculate_triangle_depth_order();
                assert(depthOrder.size() == triangles.size());
                
                const size_t numVertices = dstpts.size();
                
                // For each triangle in depth order, call the callback
                for (int64_t tri_idx : depthOrder)
                {
                    if (tri_idx < 0 || tri_idx >= static_cast<int64_t>(triangles.size()))
                        continue; // Skip invalid triangle index
                    
                    const Triangle& triangle = triangles[tri_idx];
                    
                    if (triangle[0] >= numVertices || triangle[1] >= numVertices || triangle[2] >= numVertices)
                        continue; // Skip triangle with invalid vertex indices
                    
                    callback(triangle, srcpts, dstpts, vertexOffset);
                }
                
                vertexOffset += dstpts.size();
            }
        }

        const std::vector<std::shared_ptr<PuppetT>>& PuppetStroker::puppets() const
        {
            return _puppets;
        }
        
        void PuppetStroker::setAutoPinEnabled(bool enabled)
        {
            std::lock_guard<std::mutex> lock(_mainThreadMutex);
            _queuedActionsJobGroup.wait();
            _autoPinEnabled = enabled;
            // Auto-pin values are passed to puppets at drag time via add_handle()
        }

        void PuppetStroker::setAutoPinFraction(double fraction)
        {
            std::lock_guard<std::mutex> lock(_mainThreadMutex);
            _queuedActionsJobGroup.wait();
            _autoPinFraction = std::clamp(fraction, 0.0, 1.0);
            // Auto-pin values are passed to puppets at drag time via add_handle()
        }

        const PuppetT& PuppetStroker::selectedPuppet() const
        {
            return *_puppets[_puppetStackingOrder.back()];
        }

        PuppetDynamicIterationStrategy& PuppetStroker::iterationStrategy()
        {
            return _iterationStrategy;
        }

        const PuppetDynamicIterationStrategy& PuppetStroker::iterationStrategy() const
        {
            return _iterationStrategy;
        }

        bool PuppetStroker::selectPuppetAtPoint(const geom::Vector2f& point)
        {
            USING_OPTIMTOOLS_MESH_TYPES(typename,Scalar,2);

            // Convert geom::Vector2f to Point for the is_point_inside call
            Point p(point.x, point.y);

            // Find the puppet that contains the given point
            // Iterate in reverse to give higher z-order puppets
            // the first opportunity to be selected
            for (int32_t i = _puppetStackingOrder.size() - 1; i >= 0; --i)
            {
                int32_t puppetIndex = _puppetStackingOrder[i];
                
                // Check if the point is inside the puppet. If yes, then select it. Otherwise, the puppet wasn't hit (yet).
                bool is_inside_puppet = _puppets[puppetIndex]->point_inside(p);
                if (!is_inside_puppet)
                {
                    // Although the click was not inside the puppet, it might still be close enough to register the hit.
                    // Note: we need both checks, because based on triangle sizes and zoom levels,
                    //       the clicked position might be further away from all vertices than the allowed threshold radius.
                    //       E.g. if the user zooms in extremely close, we should still register the hit if it's inside the mesh.
                    
                    // Check if we hit the puppet with our click. If not, move on to the next one.
                    // We allow the hit to be slightly outside the puppet and still register it.
                    const auto& mesh = _puppets[puppetIndex]->deformed_mesh();
                    Point nearest_mesh_point = puppet::PuppetGeometryUtilities<Scalar>::get_nearest_point_mesh_point(p, mesh);
                    Scalar distance_to_curr_puppet = (nearest_mesh_point - p).matrix().norm();
                    bool within_radius = (distance_to_curr_puppet <= _outsideHitRadius);
                    if (!within_radius)
                    {
                        // Puppet wasn't hit, let's check the next puppet.
                        continue;
                    }
                }

                // If the puppet is already selected (it's the last in the
                // z-order), then return true without modifying anything
                if (i == _puppetStackingOrder.size() - 1)
                {
                    return true;
                }

                // Move this puppet index to the end of the vector so it becomes the active one
                // (since selectedPuppet() returns the index at the last element)
                std::rotate(_puppetStackingOrder.begin() + i,
                            _puppetStackingOrder.begin() + i + 1,
                            _puppetStackingOrder.end());
                return true;
            }

            // If no puppet contains the point, do nothing (keep current selection)
            return false;
        }

        void PuppetStroker::beginSamplesBackgroundAction(const geom::Vector2f& point)
        {
            // Block actions from being performed on the background thread
            std::lock_guard<std::mutex> lock(_backgroundThreadMutex);
            
            USING_OPTIMTOOLS_MESH_TYPES(typename,Scalar,2);
            Point p(point.x, point.y);
            
            // Use the zoom-dependent hit radius for handle selection.
            Scalar handle_threshold_dist = _outsideHitRadius;

            // Add handle dragger to selected puppet, passing auto-pin settings from the stroker
            _handleDragger = mutableSelectedPuppet().add_handle(p, handle_threshold_dist,
                                                                _autoPinEnabled, _autoPinFraction);
        }
        
        void PuppetStroker::addSampleBackgroundAction(const PuppetStrokeSample& sample)
        {
            // Early exit if cancelled
            // Since _cancelled is atomic, it can be checked without holding a mutex
            if (_cancelled.load())
            {
                return;
            }

            // Block actions from being performed on the background thread
            std::lock_guard<std::mutex> lock(_backgroundThreadMutex);
            
            PuppetT& puppet = mutableSelectedPuppet();
            if (!puppet.is_valid_dragger(_handleDragger))
            {
                return;
            }

            // Determine max number of iterations to perform for this sample
            double newMaxIterations = _iterationStrategy.sampleWillProcess();
            puppet.set_max_iterations(newMaxIterations);

            USING_OPTIMTOOLS_MESH_TYPES(typename,Scalar,2);
            Point p(sample.position.x, sample.position.y);

            // Move dragger and iterate warper (expensive operations)
            puppet.move_dragger(_handleDragger, p);
            int64_t iterations = puppet.iterate_warper();

            _iterationStrategy.sampleProcessed(iterations);
            
            // Call the lambda to let client know puppets have been updated
            if (_puppetsUpdated)
                _puppetsUpdated();
        }
        
        void PuppetStroker::endSamplesBackgroundAction()
        {
            // Block actions from being performed on the background thread
            std::lock_guard<std::mutex> lock(_backgroundThreadMutex);
            
            PuppetT& puppet = mutableSelectedPuppet();
            if (!puppet.is_valid_dragger(_handleDragger))
            {
                return;
            }
            
            // End dragging operation
            puppet.end_dragging();
            _handleDragger.reset();
        }
        
        std::unique_ptr<PuppetStroker::PuppetStrokerState> PuppetStroker::puppetStrokerState() const
        {
            // Block actions from being queued on the main thread
            std::lock_guard<std::mutex> lock(_mainThreadMutex);
            // Wait for all outstanding operations to complete on the background thread
            _queuedActionsJobGroup.wait();
            
            // Create a new concrete state data object
            std::unique_ptr<ConcretePuppetStrokerState> state = std::make_unique<ConcretePuppetStrokerState>();
            
            // Save the puppet stacking order vector to preserve ordering
            state->puppetIndices = _puppetStackingOrder;
            
            // Collect states from all puppets in order
            state->puppetStates.reserve(_puppets.size());                
            for (const auto& puppet : _puppets)
            {
                if (puppet)
                {
                    state->puppetStates.push_back(puppet->copyState());
                }
            }

            return state;
        }
        
        void PuppetStroker::setPuppetStrokerState(const PuppetStrokerState& state)
        {
            // Block actions from being queued on the main thread
            std::lock_guard<std::mutex> lock(_mainThreadMutex);
            // Wait for all outstanding operations to complete on the background thread
            _queuedActionsJobGroup.wait();
            
            // Cast to the concrete type to access the actual data
            const ConcretePuppetStrokerState& concreteState = static_cast<const ConcretePuppetStrokerState&>(state);
            
            // First, restore the puppet stacking order to preserve ordering
            _puppetStackingOrder = concreteState.puppetIndices;

            // Then, apply the puppet states to restore their internal state
            // Note: Auto-pin settings are NOT restored here since they are tool preferences,
            // not document state. They will be applied at the next drag via add_handle().
            for (size_t i = 0; i < _puppets.size(); ++i)
            {
                if (i < concreteState.puppetStates.size() && _puppets[i])
                {
                    _puppets[i]->setState(*concreteState.puppetStates[i]);
                }
            }
            
            if (_puppetsUpdated)
                _puppetsUpdated();
        }
    
        void PuppetStroker::setPuppetsUpdatedCallback(std::function<void()> puppetsUpdated)
        {
            _puppetsUpdated = std::move(puppetsUpdated);
        }

        PuppetT& PuppetStroker::mutableSelectedPuppet()
        {
            return *_puppets[_puppetStackingOrder.back()];
        }
    } // namespace puppet
} // namespace extensions 
