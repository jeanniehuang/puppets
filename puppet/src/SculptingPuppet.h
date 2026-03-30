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
// AdobePatentID="P12053-US"

#ifndef EXTENSIONS_PUPPET_SCULPTING_PUPPET
#define EXTENSIONS_PUPPET_SCULPTING_PUPPET

#include "optimtools/common.h"
#include <algorithm>
#include <cmath>
#include <numeric>

#include "AbstractPuppet.h"
#include "ADMMBirdWarper.h"
#include "PuppetGeometryUtilities.h"

#include "puppet_warp/puppet_base.h"
#include "puppet_warp/puppet_handle.h"
#include "puppet_warp/puppet_figure.h"
#include "puppet_warp/puppet_mesh.h"
#include "puppet_warp/puppet_utils.h"

#if LOG_SCULPTING_PERFORMANCE
#include "base/Time.h"
#endif

namespace extensions
{
    namespace puppet
    {
        using namespace optimtools;
        
        /**
         * A template class that implements a sculpting puppet system for interactive mesh deformation.
         *
         * The SculptingPuppet class provides functionality for real-time mesh deformation through handle-based
         * manipulation. It uses an ADMM (Alternating Direction Method of Multipliers) warper to compute
         * smooth deformations.
         *
         * Typical usage::
         * 1. Create a SculptingPuppet with a figure
         * 2. Add handles at desired points using add_handle()
         * 3. Move handles with move_dragger() to deform the mesh
         * 4. Call iterate_warper() to compute the deformation
         * 5. Call end_dragging() to remove the handles and rest the mesh at the new deformed position.
         * 6. Access the deformed mesh via deformed_mesh()
         */
        
        template<typename Scalar>
        class SculptingPuppet : public AbstractPuppet {
            USING_OPTIMTOOLS_MESH_TYPES(typename,Scalar,2);
            typedef Eigen::Matrix<Scalar,Eigen::Dynamic,1> Vector;
            typedef typename Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Matrix;
            typedef Eigen::SparseMatrix<Scalar> SparseMatrix;
            typedef typename SculptingPuppet<Scalar>::HandleDragger HandleDragger;
 
#pragma mark - Protected Nested Classes
        protected:
            struct DepthState {
                // Z-ordering of triangle indices for layering overlapping parts of a mesh while posing (e.g. appendages)
                std::vector<int64_t> triangle_depth_order;
                bool has_baseline{false};
                Vector orig_point_depths;
                Vector curr_laplacian_depths; // [0, 1) baseline for the current manipulation
                                              // Precomputed per-triangle depth values.
                Vector background_triangle_depths; // Used for unmoved triangles
                Vector foreground_triangle_depths; // Used for moved triangles
            };
            
            /**
             * Concrete implementation of PuppetState for SculptingPuppet.
             * This will hold the state information specific to sculpting puppets.
             */
            class SculptingPuppetState : public AbstractPuppet::PuppetState {
            public:
                PointVector baseMeshPoints;
                DepthState depth_state;
                
                SculptingPuppetState() = default;
                virtual ~SculptingPuppetState() = default;
            };
         
#pragma mark - Public Methods
        public:
            typedef PuppetFigure<Scalar,PuppetHandle> Figure;
            
            SculptingPuppet() = delete;
            
            SculptingPuppet(const Figure& figure) {
                m_figure = make_shared<Figure>(figure);
                m_base_mesh = make_shared<optimtools::PuppetMesh<Scalar> >();
                m_is_deformed_mesh_dirty = true;
            }
            
            virtual ~SculptingPuppet()
            {
            }
            
            HandleDragger add_handle(const Point& p,
                                     const double& threshold_dist,
                                     bool autoPinEnabled,
                                     double autoPinFraction) override {
                init_warper();
                
                // Clear any cached triangle order since we're starting a new manipulation
                // Note: for possible performance gains, we could be more diligent about only calculating a new
                //       depth value and ordering for the triangles only where there was a change. We might be able
                //       to get away without a full sort with a custom partial sort + insertion of newly moved triangles.
                m_depth_state.triangle_depth_order.clear();
                
                // Find the "best" handle point: if we have depth information, use point with highest z-order within radius
                // Otherwise fall back to nearest point
                Int handle_id = m_handles.size();
                // By default, -1 signifies that no vertices were find within the search radius
                Int selected_point_idx = -1;
                
                Scalar search_distance = threshold_dist;
                
                // Use existing depth information if available and valid
                if (m_depth_state.has_baseline &&
                    m_depth_state.orig_point_depths.size() == m_deformed_mesh->points().size() &&
                    m_depth_state.orig_point_depths.size() > 0) {
                    // Check if we're completely inside a triangle first. If so, pick the topmost one.
                    Int inside_point_idx = PuppetGeometryUtilities<Scalar>::find_vertex_from_topmost_triangle(
                        *m_deformed_mesh,
                        p,
                        m_depth_state.orig_point_depths
                    );
                    
                    if (inside_point_idx != -1) {
                        selected_point_idx = inside_point_idx;
                    } else {
                         // Fallback: Use depth-based selection within radius
                        selected_point_idx = PuppetGeometryUtilities<Scalar>::find_highest_point_in_radius(
                            m_deformed_mesh->points(),
                            p,
                            search_distance,
                            m_depth_state.orig_point_depths
                        );
                    }
                }
                
                if (selected_point_idx == -1) {
                    // Fall back to nearest point selection
                    selected_point_idx = PuppetGeometryUtilities<Scalar>::find_relative_point(
                        m_deformed_mesh->points(),
                        p,
                        true
                    );
                }
                
                IntVector handle_idx_vec(1);
                handle_idx_vec[0] = selected_point_idx;
                m_handles.push_back(handle_idx_vec);
                
                // Get the actual point location for offset calculation.
                Point selected_point = m_deformed_mesh->points()[selected_point_idx];
                                
                // Update the warper when the handles change.
                // We want to calculate the depths for the triangle ordering with only the user-dragged handle.
                // (I.e., the auto-pinned handle should be ignored in the depth calculation.)
                const PointVector& deformed_points = m_warper->points();
                recreate_warper(deformed_points);
                
                // Note, mesh is NOT marked dirty
                
                // Since the handle is not added exactly where the user dragged at point P, store the offset
                // so later calls to move_dragger include the offset
                Point offset = selected_point - p;
                
                // Recompute depth based on a quasi-distance calculation calculation (Laplacian system).
                // Our heuristic for a topological ordering of triangles for rendering (and selection):
                //   vertices/triangles are sorted based on distance to the user handle: closer triangles are on top.
                recompute_depth();
                
                if (autoPinEnabled && autoPinFraction <= 0.01) {
                    // pin single furthest vertex (based on computed distance)
                    add_single_auto_pin_boundary_offset(p, /*geodesic_distance*/ true);
                } else if (autoPinEnabled) {
                    // pin furthest X% of vertices
                    const Int num_points = m_depth_state.curr_laplacian_depths.size();
                    if (num_points > 0) {
                        const double clamped_fraction = std::clamp<double>(autoPinFraction, 0.0, 1.0);
                        Int pins_to_select = static_cast<Int>(std::round(clamped_fraction * num_points));
                        if (pins_to_select <= 0) {
                            // Ensure we pin at least three vertices.
                            pins_to_select = 3;
                        }
                        pins_to_select = std::min(pins_to_select, num_points);

                        std::vector<Scalar> depth_values(num_points);
                        for (Int i = 0; i < num_points; ++i) {
                            depth_values[i] = m_depth_state.curr_laplacian_depths[i];
                        }

                        const Int nth_index = pins_to_select - 1;
                        std::nth_element(depth_values.begin(),
                                         depth_values.begin() + nth_index,
                                         depth_values.end());
                        const Scalar threshold = depth_values[nth_index];
                        
                        std::vector<Int> candidate_indices;
                        candidate_indices.reserve(pins_to_select);
                        for (Int i = 0; i < num_points; ++i) {
                            if (m_depth_state.curr_laplacian_depths[i] <= threshold) {
                                candidate_indices.push_back(i);
                            }
                        }
                        
                        IntVector furthest_vertex_handles(candidate_indices.size());
                        for (Int i = 0; i < candidate_indices.size(); ++i)
                            furthest_vertex_handles[i] = candidate_indices[i];
                        
                        // Add the new pin handle at the computed location
                        m_handles.push_back(furthest_vertex_handles);
                    }
                }
                
                // Update the warper when the handles change. The computation so far shouldn't affect deformed_points, so we can reuse them.
                recreate_warper(m_warper->points());
                
                return HandleDragger(handle_id, offset);
            }
                        
            const std::vector<PointVector>& get_handle_points() const override {
                // Get handle points from warper (in normalized space) and denormalize them
                const std::vector<PointVector>& normalized_handle_points = m_warper->handle_points();
                
                // Cache denormalized handle points
                m_cached_denormalized_handle_points = normalized_handle_points;  // Copy
                
                // Denormalize all handle points
                for (auto& handle : m_cached_denormalized_handle_points) {
                    for (Int i = 0; i < handle.size(); ++i) {
                        handle[i] /= m_scale;  // Denormalize
                    }
                }
                
                return m_cached_denormalized_handle_points;
            }

            
            void move_dragger(const HandleDragger& dragger, const Point& p) override {
                assert(is_warper_init());
                assert(is_valid_dragger(dragger));
                
                // Include the offset when setting p
                set_handle_point(dragger.handle_id(), p + dragger.offset());
            }
            
            bool is_valid_dragger(const typename AbstractPuppet::HandleDragger& dragger) const override {
                return dragger.handle_id() >= 0;
            }
            
            virtual void end_dragging() override
            {
                m_handles.clear();
                m_pin_handle_ind = -1;
                reset_warper();
                // Triangle z-ordering is maintained after handle is released.
            }
            
            Int num_handles() const {
                return m_handles.size();
            }
            
            Int iterate_warper() override {
                init_warper();
                m_is_deformed_mesh_dirty = true;
                if (num_handles() >= 1) {
                    return m_warper->iterate();
                }
                
                // no handles, so just set the deformed mesh to the base mesh
                return 0;
            }
            
            const PuppetFigureShape<Scalar>& figure_shape() const override {
                return *m_figure;
            }
            
            const optimtools::PuppetMesh<Scalar>& base_mesh() const override {
                return *m_base_mesh;
            }
            
            const optimtools::PuppetMesh<Scalar>& deformed_mesh() const override {
                Assert(is_warper_init() || num_handles() == 0) << "cannot call `deformed_mesh()` before the warper is initialized" << raise;
                if (m_is_deformed_mesh_dirty) {
                    if(num_handles() >= 1) {
                        // There are handles, so update deformed mesh from warper
                        update_deformed_mesh();
                    } else {
                        // No handles, so just set the deformed mesh to the base mesh
                        m_deformed_mesh->points() = m_base_mesh->points();
                    }
                    const_cast<bool&>(m_is_deformed_mesh_dirty) = false;
                }
                return *m_deformed_mesh;
            }
            
            const PointVector& initial_points() const override {
                return m_initial_points;
            }
            
            bool point_inside(const Point& p) const override {
                // updates the deformed mesh if necessary
                deformed_mesh();
                
                // Note, deformed_mesh() returns a reference, but point_inside_mesh() needs a shared_ptr,
                // so m_deformed_mesh is passed.
                return PuppetGeometryUtilities<Scalar>::point_inside_mesh(p, m_deformed_mesh);
            }
            
            std::unique_ptr<AbstractPuppet::PuppetState> copyState() const override {
                auto state = std::make_unique<SculptingPuppetState>();
                
                // Copy the base mesh points if the base mesh exists
                if (m_base_mesh) {
                    state->baseMeshPoints = m_base_mesh->points(); // Copy the points
                    state->depth_state = m_depth_state; // Copy depth state
                }
                
                return state;
            }
            
            bool compareState(const AbstractPuppet::PuppetState& state) const override {
                constexpr double epsilon = 1e-10;
                const SculptingPuppetState* sculptingState = dynamic_cast<const SculptingPuppetState*>(&state);
                if (!sculptingState) {
                    return false;
                }
                
                // Compare base mesh points
                if (m_base_mesh && sculptingState->baseMeshPoints.size() != m_base_mesh->points().size()) {
                    return false;
                }
                
                if (m_base_mesh) {
                    const PointVector& currentPoints = m_base_mesh->points();
                    for (size_t i = 0; i < currentPoints.size(); ++i) {
                        if (!currentPoints[i].isApprox(sculptingState->baseMeshPoints[i], epsilon)) {
                            return false;
                        }
                    }
                }
                
                // Compare depth state
                const DepthState& currentDepthState = m_depth_state;
                const DepthState& otherDepthState = sculptingState->depth_state;
                
                // Compare triangle_depth_order
                if (currentDepthState.triangle_depth_order != otherDepthState.triangle_depth_order) {
                    return false;
                }
                
                // Compare has_baseline
                if (currentDepthState.has_baseline != otherDepthState.has_baseline) {
                    return false;
                }
                
                // Compare orig_point_depths
                if (currentDepthState.orig_point_depths.size() != otherDepthState.orig_point_depths.size()) {
                    return false;
                }
                for (int i = 0; i < currentDepthState.orig_point_depths.size(); ++i) {
                    if (std::abs(currentDepthState.orig_point_depths[i] - otherDepthState.orig_point_depths[i]) > epsilon) {
                        return false;
                    }
                }
                
                // Compare curr_laplacian_depths
                if (currentDepthState.curr_laplacian_depths.size() != otherDepthState.curr_laplacian_depths.size()) {
                    return false;
                }
                for (int i = 0; i < currentDepthState.curr_laplacian_depths.size(); ++i) {
                    if (std::abs(currentDepthState.curr_laplacian_depths[i] - otherDepthState.curr_laplacian_depths[i]) > epsilon) {
                        return false;
                    }
                }
                
                // Compare background_triangle_depths
                if (currentDepthState.background_triangle_depths.size() != otherDepthState.background_triangle_depths.size()) {
                    return false;
                }
                for (int i = 0; i < currentDepthState.background_triangle_depths.size(); ++i) {
                    if (std::abs(currentDepthState.background_triangle_depths[i] - otherDepthState.background_triangle_depths[i]) > epsilon) {
                        return false;
                    }
                }
                
                // Compare foreground_triangle_depths
                if (currentDepthState.foreground_triangle_depths.size() != otherDepthState.foreground_triangle_depths.size()) {
                    return false;
                }
                for (int i = 0; i < currentDepthState.foreground_triangle_depths.size(); ++i) {
                    if (std::abs(currentDepthState.foreground_triangle_depths[i] - otherDepthState.foreground_triangle_depths[i]) > epsilon) {
                        return false;
                    }
                }
                
                return true;
            }
            
            void setState(const AbstractPuppet::PuppetState& state) override {
                // Cast to our specific state type
                const SculptingPuppetState* sculptingState = static_cast<const SculptingPuppetState*>(&state);
                
                // Set base mesh points
                m_base_mesh->points() = sculptingState->baseMeshPoints;
                
                // Set depth state
                m_depth_state = sculptingState->depth_state;
                
                // Initialize warper and deformed mesh from the restored base mesh
                init_warper_and_deformed_from_base();
            }
            
            // Triangle size parameter
            void set_triangle_size(double size) {
                m_triangle_size = size;
            }
            
            // Bird loss w parameter
            void set_bird_w(double w) {
                m_bird_w = w;
            }
            
            // Bird loss s parameter
            void set_bird_s(double s) {
                m_bird_s = s;
            }
            
            // Max iterations parameter
            void set_max_iterations(double max_iterations) override {
                m_max_iterations = max_iterations;
                if (m_warper) {
                    m_warper->set_max_iterations(max_iterations);
                }
            }
            
            // Recompute depth based on a quasi-distance calculation calculation (Laplacian system).
            // Our heuristic for a topological ordering of triangles for rendering (and selection):
            //   vertices/triangles are sorted based on distance to the user handle: closer triangles are on top.
            void recompute_depth() {
                // Recompute a baseline depth for the new handle
                Int num_points = m_warper->rest_points().size();
                Int num_handles = 0;
                for (const auto& hv : m_warper->handles())
                    num_handles += hv.size();
                
                // Compute current vertex depths using Laplacian system
                Vector xeq = Vector::Zero(num_handles);  // Zero depth at handles.
                Vector b = Vector::Ones(num_points);     // Uniform source
                Vector baseline_depths;
                m_warper->solve_laplacian_system_to(baseline_depths, b, xeq);
                normalize01_inplace(baseline_depths); // rescale to [0, 1]
                
                if (!m_depth_state.has_baseline || m_depth_state.orig_point_depths.size() != num_points) {
                    // For first handle: store baseline as the current depths
                    m_depth_state.orig_point_depths = baseline_depths; // [0, 1)
                    m_depth_state.curr_laplacian_depths = baseline_depths;
                    m_depth_state.has_baseline = true;
                } else {
                    // New handle added: push previous ordering down to [-1, 0)
                    push_down_depths_inplace(m_depth_state.orig_point_depths); // [-1, 0)
                    // Save the new [0, 1) baseline for this manipulation
                    m_depth_state.curr_laplacian_depths = baseline_depths; // [0, 1)
                }
                
                // Precompute per-triangle depths for background (unmoved) and foreground (moved) triangles
                const TriangleVector& triangles = m_warper->triangles();
                Int num_triangles = triangles.size();
                
                m_depth_state.background_triangle_depths.resize(num_triangles);
                m_depth_state.foreground_triangle_depths.resize(num_triangles);
                
                // Helper lambda to compute average depth of triangle vertices
                auto average_triangle_depth = [](const Vector& depths, const Triangle& tri) -> Scalar {
                    return (depths[tri[0]] + depths[tri[1]] + depths[tri[2]]) / Scalar(3);
                };
                
                for (Int i = 0; i < num_triangles; ++i) {
                    const Triangle& tri = triangles[i];
                    
                    // Compute average depth for background (unmoved) triangles
                    m_depth_state.background_triangle_depths[i] = 
                        average_triangle_depth(m_depth_state.orig_point_depths, tri);
                    
                    // Compute average depth for foreground (actively moved) triangles
                    m_depth_state.foreground_triangle_depths[i] = 
                        average_triangle_depth(m_depth_state.curr_laplacian_depths, tri);
                }
            }
            
            /** Get triangle order for handling overlapping parts of the puppet.
                The order is based on a quasi-distance computed inside ADMMBirdWarper.
                Triangles closer to handles will be rendered on top (appear later in the sorted list of triangle indices).
                Returns triangle indices sorted by depth for proper layering. */
            std::vector<int64_t> calculate_triangle_depth_order() override {
                if (!is_warper_init()) {
                    // No warper, return sequential order
                    std::vector<int64_t> sequential_indices(base_mesh().triangles().size());
                    std::iota(sequential_indices.begin(), sequential_indices.end(), 0);
                    return sequential_indices;
                }
                
                Int num_points = m_warper->rest_points().size();
                Int num_triangles = m_warper->triangles().size();
                Int num_handles = 0;
                for (const auto& hv : m_warper->handles())
                    num_handles += hv.size();
                                
                // Create sequential ordering
                std::vector<int64_t> triangle_indices(num_triangles);
                std::iota(triangle_indices.begin(), triangle_indices.end(), 0);
                
                // Note: We're lumping together negative values with the num_handles == 0 condition.
                //       The reasoning for this is that we assume that num_handles is non-negative, but it's technically
                //       implemented as a signed integer value. So in case there's a case for a value of -1 or similar,
                //       it still makes sense to return a reasonable ordering instead of running into some broken distance computation.
                // If no handles exist, the distance calculation would make no sense to run, ...
                if (num_handles <= 0) {
                    if (!m_depth_state.triangle_depth_order.empty() && m_depth_state.triangle_depth_order.size() == static_cast<size_t>(num_triangles)) {
                        // ...  but we have a "cached" order from previous manipulation, so let's use it
                        return m_depth_state.triangle_depth_order;
                    }
                    // ... and no suitable ordering is stored. Let's return a basic sequential order.
                    m_depth_state.triangle_depth_order = triangle_indices;
                    return triangle_indices;
                }
                
                // Safety fallbacks if depth vectors weren't initialized properly
                if (m_depth_state.orig_point_depths.size() != num_points) {
                    Vector tmp = Vector::Zero(num_points);
                    m_depth_state.orig_point_depths = tmp;
                    m_depth_state.has_baseline = false;
                }
                if (m_depth_state.curr_laplacian_depths.size() != num_points) {
                    // Baseline fallback
                    Vector xeq = Vector::Zero(num_handles);  // Zero depth at handles.
                    Vector b = Vector::Ones(num_points);     // Uniform source
                    m_warper->solve_laplacian_system_to(m_depth_state.curr_laplacian_depths, b, xeq);
                    normalize01_inplace(m_depth_state.curr_laplacian_depths);
                }
                
                // Check if vertex should be raised to foreground
                static constexpr double w_tol = 1e-3;
                const Eigen::VectorXd& point_w = m_warper->point_w();
                for (Int i = 0; i < num_points; ++i) {
                    if (point_w[i] > w_tol) {
                        // If vertex has moved, then stick to having this part raised into the foreground.
                        // Note: even if the given triangle was returned to its initial position (thus become "unmoved"), this logic
                        //       "remembers" that it should be on top, and thus stays permanently on top while dragging the current handle.
                        m_depth_state.orig_point_depths[i] = m_depth_state.curr_laplacian_depths[i];
                    }
                }

                // Compute triangle depths ensuring consistency within each triangle
                // If any vertex of a triangle has moved, put the entire triangle in the top layer
                Vector triangle_depths(num_triangles);
                const TriangleVector& triangles = m_warper->triangles();
                
                for (Int i = 0; i < num_triangles; ++i) {
                    const Triangle& curr_triangle = triangles[i];
                    
                    // Check if any vertex of this triangle has moved (since started dragging the current handle)
                    bool is_foreground_triangle = (m_depth_state.orig_point_depths[curr_triangle[0]] >= 0) ||
                                                  (m_depth_state.orig_point_depths[curr_triangle[1]] >= 0) ||
                                                  (m_depth_state.orig_point_depths[curr_triangle[2]] >= 0);

                    if (is_foreground_triangle) {
                        // use current Laplacian depths for the entire triangle,
                        // (pre)computed as an average of the Laplacian depths from the 3 vertices making up this triangle
                        triangle_depths[i] = m_depth_state.foreground_triangle_depths[i];
                    } else {
                        // None of the vertices making up this triangle have left their initial positions since the current dragging began.
                        // Use depths from the background layer.
                        triangle_depths[i] = m_depth_state.background_triangle_depths[i];
                    }
                }
                
                // Sort triangle indices based on the approx. distance from the handle.
                // Note: for a performance gain, we could improve this via a partial sort only,
                //       using our knowledge that the triangles that didn't move are already
                //       consistently ordered amongst themselves.
                std::sort(triangle_indices.begin(), triangle_indices.end(),
                    [&](int64_t a, int64_t b) {
                        return triangle_depths[a] < triangle_depths[b];
                    });
                
                // Cache the computed order for use after handle release
                m_depth_state.triangle_depth_order = triangle_indices;
                
                return triangle_indices;
            }
            
#pragma mark - Protected Methods
        protected:
            
            bool is_warper_init() const {
                return (bool)m_warper;
            }
            
            /**
             * Recreates the warper with new base points and handle vertex sets.
             */
            void recreate_warper(const PointVector& base_points) {
                // Extract handle vertices from m_handles
                std::vector<IntVector> handle_vert_sets;
                for (const auto& handle : m_handles) {
                    handle_vert_sets.push_back(handle);
                }
                
                // Recreate warper
                m_warper = make_shared<ADMMBirdWarper<Scalar>>(
                                                               base_points,
                                                               m_base_mesh->triangles(),
                                                               handle_vert_sets,
                                                               m_bird_s,
                                                               m_bird_w
                                                               );
                
                // Set max iterations
                m_warper->set_max_iterations(m_max_iterations);
            }
            
            /**
             * Initialize warper and deformed mesh from the current base mesh.
             * Assumes m_base_mesh is already set up with the correct points and triangles.
             */
            void init_warper_and_deformed_from_base() {
                // Normalize mesh points to [-1,1] range for better numerical stability
                PointVector normalized_base_points = m_base_mesh->points(); // copy
                normalize_points_inplace(normalized_base_points);
                
                // Update the warper with the new points
                recreate_warper(normalized_base_points);
                
                // Initialize deformed mesh from base mesh
                m_deformed_mesh = make_shared<optimtools::PuppetMesh<Scalar> >(
                                                                   make_shared<PointVector>(m_base_mesh->points()), // copy for warper
                                                                   m_base_mesh->triangles_ptr(),
                                                                   m_base_mesh->point_handles_ptr()
                                                                   );
                m_is_deformed_mesh_dirty = true;
            }
            
            /**
             * Adds a new handle to the puppet at the specified point.
             * The handle is associated with the nearest point on the deformed mesh.
             * Returns a HandleDragger object for manipulating the handle.
             */
            void set_handle_point(Int handle_id, const Point& p) {
                Assert(is_warper_init()) << "cannot call `set_handle_point()` before the warper is initialized" << raise;
                
                if (handle_id < 0 || handle_id >= m_handles.size()) {
                    Assert(false) << "handle_id not found in m_handles" << raise;
                    return;
                }
                
                // Find the handle index in the warper's handle list
                const std::vector<IntVector>& warper_handles = m_warper->handles();
                Int handle_ind = -1;
                
                // Find which handle in the warper corresponds to this handle_id
                for (Int i = 0; i < warper_handles.size(); ++i) {
                    if (warper_handles[i].size() == m_handles[handle_id].size() &&
                        (warper_handles[i] == m_handles[handle_id]).all()) {
                        handle_ind = i;
                        break;
                    }
                }
                
                Assert(handle_ind >= 0) << "handle not found in warper handles" << raise;
                
                PointVector& handle_points = m_warper->handle_points()[handle_ind];
                Assert(handle_points.size() == 1) << "Multi-point handle not implemented." << raise;
                handle_points[0] = p * m_scale;
            }
                        
            /**
             * Initialize warper with base mesh and handles
             */
            void init_warper() {
                if(!is_warper_init()) {
                    // Compute base mesh
                    compute_base_mesh();
                    
                    // Capture the initial points of the mesh before any warping
                    m_initial_points = m_base_mesh->points(); // copy
                    
                    // Initialize warper and deformed mesh from the base mesh
                    init_warper_and_deformed_from_base();
                }
            }
            
            /**
             * Resets the warper.
             * Updates the puppet’s base mesh to match the warper’s current deformed points,
             * effectively making the deformed mesh the new base mesh.
             * This commits the current warper’s effect to the puppet, allowing a new warper
             * to be created without the need to retain the handles from the current warper.
             */
            void reset_warper() {
                // Denormalize the current warper points and set the base mesh to the denormalized points
                m_base_mesh->points() = m_warper->points(); // copy
                denormalize_points_inplace(m_base_mesh->points());
                
                // Note, the triangles are the same in the warper and mesh so we don't copy them
                
                // Initialize warper and deformed mesh from the updated base mesh
                init_warper_and_deformed_from_base();
            }
            
            /**
             * Compute base mesh from figure shape
             */
            void compute_base_mesh() {

#if LOG_SCULPTING_PERFORMANCE
                base::PerformanceTimer timer("\tSculpting: compute base mesh");
#endif
                
                auto mesher = BasicPuppetFigureMesher<Scalar>(m_figure);

#if LOG_SCULPTING_PERFORMANCE
                printf("\tSculpting: m_triangle_size: %f, figure_extent: %f, triangle_size_scale: %f\n",
                      static_cast<double>(m_triangle_size),
                      static_cast<double>(figure_extent),
                      static_cast<double>(triangle_size_scale));
#endif
                
                // Set meshing quality
                typename BasicPuppetFigureMesher<Scalar>::Quality quality = mesher.quality();
                quality.set_triangle_size_scale(m_triangle_size);
                mesher.set_quality(quality);
                
                m_base_mesh = mesher.solve();
                
#if LOG_SCULPTING_PERFORMANCE
                printf("\tSculpting: base mesh triangles: %zu, vertices: %zu\n", m_base_mesh->triangles().size(), m_base_mesh->points().size());
#endif
            }
            
            /**
             * Normalize mesh points to [-1,1] range for better numerical stability.
             * This method scales all points so they fit within a unit square centered at the origin.
             * The scaling factor is stored in m_scale for later denormalization.
             */
            void normalize_points_inplace(PointVector& points) {
                // Compute scale Pfactor to normalize points to [-1,1] range
                Eigen::AlignedBox2d bbox{points[0], points[0]};
                for(Int i = 1; i < points.size(); ++i) {
                    bbox.extend(points[i].matrix());
                }
                m_scale = 2.0 / bbox.diagonal().maxCoeff();
                
                // Scale points to [-1,1] range
                for(Int i = 0; i < points.size(); ++i) {
                    points[i] *= m_scale;
                }
            }
            
            // Denormalize points in-place by dividing by m_scale.
            void denormalize_points_inplace(PointVector& points) const {
                for(Int i = 0; i < points.size(); ++i) {
                    points[i] /= m_scale;
                }
            }
            
            // Normalize to [0,1) range in-place (not [0,1] due to the epsilon term)
            static void normalize01_inplace(Vector& v) {
                double vmin = v.minCoeff();
                double vmax = v.maxCoeff();
                v = (v.array() - vmin) / (vmax - vmin + 1e-8);
            }
            
            // Transform range to [-1,0) range, used for "pushing down" calculated depth values
            static void push_down_depths_inplace(Vector& v) {
                normalize01_inplace(v);
                v = v.array() - 1.0;
            }
            
            // Copy warper points to deformed mesh and unscale them
            void update_deformed_mesh() const {
                PointVector& points = m_deformed_mesh->points();
                points = m_warper->points();
                denormalize_points_inplace(points);
            }
            
            bool auto_pin_set() {
                return m_pin_handle_ind != -1;
            }
            
            /**
             * Adds a single auto pin handle at the furthest location from a location offset inward from the boundary.
             * If auto pinning is enabled and no auto pin has been set, this method finds the furthest
             * mesh vertex from the given point p, computes the inward boundary tangent at that vertex,
             * and places a pin handle a fixed distance (edge length) inward from the boundary.
             *
             * @param p The point from which to compute the furthest boundary vertex and inward offset.
             * @param geodesic_distance Calculates distance along the shape if true, uses Euclidean distance otherwise.
             */
            void add_single_auto_pin_boundary_offset(const Point& p, bool geodesic_distance) {
                
                if (auto_pin_set())
                    return;
                
                // Get a reference to the current deformed mesh points
                PointVector& points = m_deformed_mesh->points();
                
                Int furthest_point_idx = 0;
                
                if (geodesic_distance && m_depth_state.curr_laplacian_depths.size() == points.size()) {
                    // Find index of minimum depth using std::min_element
                    const auto& depths = m_depth_state.curr_laplacian_depths;
                    furthest_point_idx = std::min_element(depths.begin(), depths.end()) - depths.begin();
                } else {
                    // Euclidean furthest search
                    furthest_point_idx = PuppetGeometryUtilities<Scalar>::find_relative_point(points, p, false);
                }
                
                IntVector furthest_handle_idx_vec(1, furthest_point_idx);
                
                // Get the coordinates of the furthest point
                const Point furthest_point = points[furthest_point_idx];
                
                // Use the furthest vertex index to find the inward boundary tangent at that vertex
                Point inward_tangent = PuppetGeometryUtilities<Scalar>::find_inward_boundary_tangent(furthest_point_idx, deformed_mesh());
                
                // Compute the average edge length of the mesh to determine how far to move inward
                Scalar average_edge_length = PuppetGeometryUtilities<Scalar>::get_average_edge_length(deformed_mesh());
                // Move inward from the boundary by average edge length along the inward tangent
                Scalar scale_factor = 1.0; // TY_TODO: this needs to be tuned relative to the triangle size.
                Point inward_point = furthest_point + inward_tangent * average_edge_length * scale_factor;
                
                // Find the nearest mesh vertex to the computed inward point
                furthest_handle_idx_vec = PuppetGeometryUtilities<Scalar>::nearest_point_ind_vec(m_deformed_mesh->points(), inward_point);
                
                // Add the new pin handle at the computed location
                m_handles.push_back(furthest_handle_idx_vec);
                // Set the pin handle index
                m_pin_handle_ind = m_handles.size()-1;
            }
            
#pragma mark - Protected members
            shared_ptr<Figure> m_figure;
            shared_ptr<ADMMBirdWarper<Scalar>> m_warper;
            
            // Stores the initial (normalized) mesh points used to initialize the warper.
            // These points do not change after initialization.
            PointVector m_initial_points;
            
            // The base mesh used for initializing the warper and as the undeformed reference mesh.
            shared_ptr<optimtools::PuppetMesh<Scalar> > m_base_mesh;
            
            // Deformed mesh. Updated by copy_warper_points_to_deformed_mesh() when m_is_deformed_mesh_dirty is true
            shared_ptr<optimtools::PuppetMesh<Scalar> > m_deformed_mesh;
            
            // Stores the handle vertex indices for each handle.
            // Each handle may correspond to one or more mesh vertices (for multi-point handles).
            std::vector<IntVector> m_handles;
            
            // Index of the handle that is pinned (if auto pin is enabled). -1 indicates no pin.
            Int m_pin_handle_ind {-1};
            
            // True indicates m_deformed_mesh needs to be updated from m_warper
            bool m_is_deformed_mesh_dirty {false};
            
            // Scale factor to normalize mesh points to [-1,1] range for numerical stability
            // Used in init_warper() to scale points before warping, and unscale after warping
            double m_scale = 1.0;
            
            // Triangle size
            double m_triangle_size = 10.0;
            static constexpr double k_min_triangle_size_scale {0.01};
            static constexpr double k_max_triangle_size_scale {4.0};

            // bird loss parameters
            double m_bird_w{1.0};
            double m_bird_s{1.0};
            
            // Max iterations parameter
            double m_max_iterations{200};
            
            DepthState m_depth_state;
            
            // Cached denormalized handle points (mutable since get_handle_points is const)
            mutable std::vector<PointVector> m_cached_denormalized_handle_points;
        };
    }
}

#endif /* EXTENSIONS_PUPPET_SCULPTING_PUPPET */
