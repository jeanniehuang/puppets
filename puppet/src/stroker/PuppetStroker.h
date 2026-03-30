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

#ifndef EXTENSIONS_PUPPET_PUPPETSTROKER
#define EXTENSIONS_PUPPET_PUPPETSTROKER

#include <atomic>
#include <functional>
#include <mutex>

#include "PuppetDynamicIterationStrategy.h"
#include "PuppetStrokeSample.h"
#include "PuppetTypes.h"
#include "base/threads/Threads"
#include "geom/Geom"

#include "optimtools/mesh/common.h"

namespace gpu
{
    class VertexBuffer;
    class IndexBuffer;
}

namespace extensions
{
    namespace puppet
    {
        // Import mesh types for use in method signatures
        using namespace optimtools;
        USING_OPTIMTOOLS_MESH_TYPES(typename, Scalar, 2);

        /**
         * PuppetStroker encapsulates the logic for incrementally modifying a collection of puppets across multiple threads
         * while dynamically adjusting fidelity to maintain adequate performance.
         *
         * This class internally manages thread safety as long as the client invokes the APIs on the correct thread.
         *
         * Thread 1 (suggested to be the 'main thread'):
         *      - Responsible for pushing the samples to the stroker via the begin / add / end methods
         *
         * Thread 2 (suggested to be the 'render thread'):
         *      - May retrieve the intermediate warp information from the stroker at any time
         *
         * Thread 3 (can be any background thread):
         *      - Passed in as the job dispatcher by the client
         *      - Responsible for performing the warp operations
         *
         * Any thread can capture or set the state of this stroker to support an undo / redo pattern.
         */
        class PuppetStroker
        {
        public:
            class Tests;

            /**
             * Construct a PuppetStroker with the specified job dispatcher for background operations.
             * 
             * @param jobDispatcher The job dispatcher used to execute puppet operations on background threads
             */
            PuppetStroker(std::shared_ptr<volatile base::JobDispatcher> jobDispatcher);
            
            /**
             * Initialize the stroker with the provided collection of puppets.
             *
             * Should be invoked on the main thread
             *
             * @param puppets Vector of puppet objects to be managed by this stroker
             */
            void initialize(const std::vector<std::shared_ptr<PuppetT>>& puppets);
            
            /**
             * Release the references to the puppets used by the stroker.
             *
             * Should be invoked on the main thread
             */
            void releasePuppets();
            
            /**
             * Submit work to begin puppet deformation at the provided sample location.
             * This method selects the appropriate puppet and starts a new stroke operation.
             * 
             * Should be invoked on the main thread
             *
             * @param sample The initial sample position and time to begin the stroke
             * @param hitRadius Radius for hit detection allowing hits slightly outside the mesh
             * @return True if a puppet was successfully selected and deformation began, false otherwise
             */
            bool beginSamples(const PuppetStrokeSample& sample, Scalar hitRadius = 0);
            
            /**
             * Submit work to drag the selected puppet towards the provided sample position.
             * This continues an ongoing stroke operation by moving the puppet's handle.
             *
             * Should be invoked on the main thread
             *
             * @param sample The target sample position and time for the puppet deformation
             */
            void addSample(const PuppetStrokeSample& sample);
            
            /**
             * Submit work to end the current puppet deformation stroke.
             * This finalizes the stroke operation and cleans up any active handles. This
             * method blocks until any outstanding work from beginSamples or
             * addSample is completed.
             *
             * Once work has been queued via beginSamples or addSample, the client
             * must call endSamples before allowing this object to be destroyed.
             *
             * Should be invoked from the main thread
             */
            void endSamples();
            
            /**
             * Retrieve the current warp information as GPU-ready vertex and index buffers.
             * This method creates buffers from the current puppet meshes for rendering.
             * 
             * When the total vertex count would cause uint16_t index overflow (>65535), this method
             * automatically switches to a vertex-only approach where vertices are duplicated per 
             * triangle and the index buffer is set to nullptr.
             *
             * @param outVertices Reference to receive the generated vertex buffer
             * @param outIndices Reference to receive the generated index buffer (nullptr if using vertex-only approach due to overflow)
             */
            void createWarpBuffers(std::unique_ptr<gpu::VertexBuffer>& outVertices,
                                   std::unique_ptr<gpu::IndexBuffer>& outIndices) const;
            
            /**
             * Retrieve the list of puppets managed by this stroker.
             * Should only be called from outside any begin/endSamples workflow, or
             * from within the 'puppetsUpdated' lambda callback.
             */
            const std::vector<std::shared_ptr<PuppetT>>& puppets() const;
        
            /**
             * Enable or disable automatic pinning across all puppets.
             */
            void setAutoPinEnabled(bool enabled);
            bool autoPinEnabled() const { return _autoPinEnabled; }
            
            /**
             * Set the fraction of vertices to automatically pin (furthest X%) for all puppets.
             */
            void setAutoPinFraction(double fraction);
            double autoPinFraction() const { return _autoPinFraction; }
            
            /**
             * Get read-only access to the currently selected puppet.
             *
             * Should only be invoked when there is no potential for outstanding work, such as
             * before calling beginSamples or after calling endSamples()
             *
             * @return Const reference to the selected puppet object
             */
            const PuppetT& selectedPuppet() const;
            
            /**
             * Get access to the iteration strategy for maintaining low latency
             *
             * Should only be invoked when there is no potential for outstanding work, such as
             * before calling beginSamples or after calling endSamples()
             *
             * @return Reference to the dynamic iteration strategy
             */
            PuppetDynamicIterationStrategy& iterationStrategy();
            const PuppetDynamicIterationStrategy& iterationStrategy() const;

            /**
             * Opaque state object that encapsulates the current puppet stroke state.
             */
            class PuppetStrokerState
            {
            public:
                virtual ~PuppetStrokerState() = default;
            };

            /**
             * Creates an opaque state object that encapsulates the current puppet stroke state.
             * This includes the puppet ordering and internal mesh states for undo/redo operations.
             * This method blocks until outstanding work is completed.
             *
             * Can be called from any thread
             *
             * @return A unique_ptr to a PuppetStrokerState containing the current state
             */
            std::unique_ptr<PuppetStrokerState> puppetStrokerState() const;

            /**
             * Applies the given state to the current stroker instance.
             * Replaces the _puppets vector and applies the puppet states to restore internal mesh state.
             * This method blocks until outstanding work is completed.
             *
             * Can be called from any thread, but should not be invoked during an active stroke
             *
             * @param state The state to restore to
             */
            void setPuppetStrokerState(const PuppetStrokerState& state);
            
            /**
             * When the stroker updates the puppet meshes, either by calls to begin/add/endSamples() or
             * by calls to setPuppetStrokerState(), it will call the provided lambda. Note that this lambda is
             * not invoked by calls to initialize().
             *
             * Setter should be invoked on the main thread.
             *
             * @param puppetsUpdated The lambda to call when the puppet meshes are updated.
             */
            void setPuppetsUpdatedCallback(std::function<void()> puppetsUpdated);

        private:
            // Thread synchronization
            mutable std::mutex _mainThreadMutex;  // Protects main thread operations
            mutable std::mutex _backgroundThreadMutex;  // Protects background thread operations
            std::shared_ptr<volatile base::JobDispatcher> _jobDispatcher; // Background thread
            mutable base::JobGroup _queuedActionsJobGroup; // Tracks background operations
            
            // Cancellation flag - set during endSamples() to skip remaining work
            std::atomic<bool> _cancelled{false};

            // The collection of puppets managed by this stroker.
            std::vector<std::shared_ptr<PuppetT>> _puppets;
            
            // Puppets have a stacking order that is maintained by a corresponding
            // vector of indices. Order in this index vector maps to z-order for the
            // corresponding puppets. The last entry specifies a puppet that is
            // considered 'selected' while actively dragging.
            std::vector<int32_t> _puppetStackingOrder;

            // The active handle
            AbstractPuppet::HandleDragger _handleDragger;

            // The strategy that adjusts properties such as maximum number of iterations
            // to maintain low latency
            PuppetDynamicIterationStrategy _iterationStrategy;
            
            // Allow puppet drag to start slightly outside of the actual mesh.
            // This value is set via beginSamples() based on the view transform.
            Scalar _outsideHitRadius {0.0f}; // 0 disables near-miss selection
            
            // Auto-pin settings are the single source of truth for auto-pinning within
            // sketch-extension. These values are passed to puppets at drag time via add_handle().
            //
            // NOTE: Auto-pin settings are intentionally NOT included in PuppetStrokerState
            // and are NOT subject to undo/redo. Auto-pin is a tool preference (tied to the
            // UI toggle button), not document state. When setPuppetStrokerState() restores
            // a memento during undo/redo, these values remain unchanged to maintain UI consistency.
            bool _autoPinEnabled {true};
            double _autoPinFraction {0.20}; // Fraction of furthest X% vertices to auto-pin
            
            // The lambda to call when the puppets are updated
            std::function<void()> _puppetsUpdated;

            // Helper methods (called on main thread)
            bool selectPuppetAtPoint(const geom::Vector2f& point);

            // Helper methods (called on background thread)
            void beginSamplesBackgroundAction(const geom::Vector2f& point);
            void addSampleBackgroundAction(const PuppetStrokeSample& sample);
            void endSamplesBackgroundAction();

            // Helper methods (called on render thread)
            // Create GPU buffers from puppet data (automatically handles uint16_t overflow)
            void createWarpBuffers(const std::vector<std::shared_ptr<PuppetT>>& puppets,
                                   const std::vector<int32_t>& puppetStackingOrder,
                                   std::unique_ptr<gpu::VertexBuffer>& outVertices,
                                   std::unique_ptr<gpu::IndexBuffer>& outIndices) const;

            // Create vertex buffer without indices (redundant vertices per triangle)
            void createWarpBufferWithoutIndices(const std::vector<std::shared_ptr<PuppetT>>& puppets,
                                                const std::vector<int32_t>& puppetStackingOrder,
                                                std::unique_ptr<gpu::VertexBuffer>& outVertices) const;

            // Create vertex and index buffers with shared vertices
            void createWarpBuffersWithIndices(const std::vector<std::shared_ptr<PuppetT>>& puppets,
                                              const std::vector<int32_t>& puppetStackingOrder,
                                              std::unique_ptr<gpu::VertexBuffer>& outVertices,
                                              std::unique_ptr<gpu::IndexBuffer>& outIndices) const;

            // Iterate through all triangles in depth order, calling callback for each valid triangle
            void forEachTriangleInDepthOrder(const std::vector<std::shared_ptr<PuppetT>>& puppets,
                                             const std::vector<int32_t>& puppetStackingOrder,
                                             std::function<void(const Triangle&, const PointVector&, const PointVector&, uint16_t)> callback) const;

            // Get mutable access to the currently selected puppet.
            PuppetT& mutableSelectedPuppet();
        };
        
    } // namespace puppet
} // namespace extensions

#endif // EXTENSIONS_PUPPET_PUPPETSTROKER 
