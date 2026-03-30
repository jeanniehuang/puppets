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

#ifndef EXTENSIONS_PUPPET_ABSTRACTPUPPET
#define EXTENSIONS_PUPPET_ABSTRACTPUPPET

#include "optimtools/mesh/common.h"
#include "puppet_warp/puppet_mesh.h"

#include <memory>
#include <optional>

namespace extensions
{
    namespace puppet
    {
        using namespace optimtools;
        
        /**
         * Abstract base class defining the interface for puppet implementations.
         * This class defines the core operations that all puppet types must support.
         */
        class AbstractPuppet {
            USING_OPTIMTOOLS_MESH_TYPES(typename,double,2);
            
        public:
            /**
             * Opaque base class for puppet state that can be overridden by derived classes.
             * This allows derived classes to define their own state representation.
             */
            class PuppetState {
            public:
                virtual ~PuppetState() = default;
            };
            typedef PuppetFigure<double,PuppetHandle> Figure;
            
            /**
             * A utility class to store the information required to drag a handle
             * around.
             */
            class HandleDragger
            {
            public:
                HandleDragger()
                {
                    reset();
                }
                
                void reset() {
                    m_handle_id = -1;
                }
                
                /**
                 * Constructs a HandleDragger
                 *
                 * @param handle_id The ID of the handle to be dragged.
                 * @param offset The offset from the user's drag position to the position of the handle
                 */
                HandleDragger(Int handle_id, Point offset) :
                m_handle_id(handle_id),
                m_offset(offset)
                {}
                
                const Int& handle_id() const
                {
                    return m_handle_id;
                }
                
                const Point& offset() const
                {
                    return m_offset;
                }
                
            private:
                Int m_handle_id;
                
                Point m_offset;
            };
            
            virtual ~AbstractPuppet() = default;
            
            /**
             * Creates a copy of the current puppet state.
             * @return A unique_ptr to a PuppetState object containing the current state
             */
            virtual std::unique_ptr<PuppetState> copyState() const = 0;
            
            /**
             * Returns true if the current puppet state is equal to the given state.
             */
            virtual bool compareState(const PuppetState& state) const = 0;
            
            /**
             * Restores the puppet to the given state.
             * @param state The state to restore to
             */
            virtual void setState(const PuppetState& state) = 0;
            
            /**
             * Adds a new handle to the puppet at the specified point.
             * The handle is associated with the nearest point on the deformed mesh.
             * Returns a HandleDragger object for manipulating the handle.
             * @param p The point where the handle should be added
             * @param threshold_dist Distance threshold for point selection
             * @param autoPinEnabled Whether to automatically pin vertices furthest from the handle
             * @param autoPinFraction Fraction of vertices (furthest X%) to pin when auto-pinning
             */
            virtual HandleDragger add_handle(const Point& p,
                                             const double& threshold_dist,
                                             bool autoPinEnabled,
                                             double autoPinFraction) = 0;
            
            /**
             * Moves a handle dragger to a new point.
             * Updates the position of the handle associated with the dragger.
             */
            virtual void move_dragger(const HandleDragger& dragger, const Point& p) = 0;
            
            /** Checks if a handle dragger is valid. */
            virtual bool is_valid_dragger(const typename AbstractPuppet::HandleDragger& dragger) const = 0;
            
            /**
             * Ends the dragging operation.
             * Clears all handles and resets the warper to its initial state.
             */
            virtual void end_dragging() = 0;
            
            /**
             * Performs one batch of iterations of the warping algorithm.
             * @return Number of iterations performed
             */
            virtual int64_t iterate_warper() = 0;
            
            /**
             * Returns true if the point is inside the puppet mesh.
             */
            virtual bool point_inside(const Point& p) const = 0;
            
            /** Returns the figure shape defining the puppet's geometry in terms of a PSLG (planar straight line graph). */
            virtual const PuppetFigureShape<double>& figure_shape() const = 0;
            
            /** Return to the base mesh before any deformation */
            virtual const PuppetMesh<double>& base_mesh() const = 0;
            
            /** Return the deformed mesh after warping operations. */
            virtual const PuppetMesh<double>& deformed_mesh() const = 0;
            
            /** Return initial points points of puppet mesh when it was first created */
            virtual const PointVector& initial_points() const = 0;
                        
            /** Return the position for all handles. */
            virtual const std::vector<PointVector>& get_handle_points() const = 0;

            /** Sets the upper limit on the number of iterations performed by the warper
                while searching for a solution */
            virtual void set_max_iterations(double max_iterations) = 0;

            /** Calculate triangle rendering order based on depth for proper layering of appendages within the same puppet.
                Returns triangle indices sorted by depth. Back-to-front: larger depth first (farther triangles behind).
            */
            virtual std::vector<int64_t> calculate_triangle_depth_order() = 0;
        };
    }
}

#endif /* EXTENSIONS_PUPPET_ABSTRACTPUPPET */
