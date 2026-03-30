#pragma once

/** \file
 * Defines some utility functions for constructing a Puppet from a BitArray.
 */


#include "optimtools/all.h"
#include "optimtools_extras/image/bit_array.h"
#include "optimtools_extras/image/bitmask_outliner.h"

BEGIN_OPTIMTOOLS_NAMESPACE

/**
 * Adds to `inout_pslg` a polygonal outline (potentially with holes) for the
 * given `component_mask`, which should have only a single connected component.
 */
template<typename Scalar, typename VertexData>
void add_bitmask_component_outline_to_pslg(
    PSLG<Scalar,VertexData>& inout_pslg,
    const BitArray& component_mask,
    const Scalar& outline_accuracy,
    typename optimtools::geometry_traits<Scalar,2>::Point mask_origin,
    bool is_add_holes
) {
    USING_OPTIMTOOLS_MESH_TYPES(typename,Scalar,2);

    // Create a simplified polygon outline, enduring that it never differs from
    // the original outline by a distance of more than `outline_accuracy`.
    // (it's a precondition that there's only one connected component in the mask).
    PointVector outline_points = outline_bitmask<Scalar>(component_mask, false);
    outline_points = douglas_peucker<Scalar>::simplify_loop(outline_points, outline_accuracy);
    
    // Translate by mask origin
    for (auto ii = 0; ii < outline_points.size(); ii++)
    {
        outline_points[ii][0] = outline_points[ii][0] + mask_origin[0];
        outline_points[ii][1] = outline_points[ii][1] + mask_origin[1];
    }

    inout_pslg.add_outline(outline_points);

    if(is_add_holes) {
        // extract a bitmask for the holes
        BitArray hole_mask = ~component_mask;
        hole_mask.flood_fill(hole_mask.min_x(), hole_mask.min_x(), false);
        
        // test if there are any holes
        bool is_hole = !hole_mask.is_constant(false);
        if(is_hole) {
            // if so, process each hole component one at a time to avoid memory issues
            hole_mask.for_each_component([&](const BitArray& hole_component_mask) {
                PointVector hole_outline_points = outline_bitmask<Scalar>(hole_component_mask, false);
                hole_outline_points = douglas_peucker<Scalar>::simplify_loop(hole_outline_points, outline_accuracy);

                // Translate by mask origin
                for (auto ii = 0; ii < hole_outline_points.size(); ii++)
                {
                    hole_outline_points[ii][0] = hole_outline_points[ii][0] + mask_origin[0];
                    hole_outline_points[ii][1] = hole_outline_points[ii][1] + mask_origin[1];
                }

                // a hole must be oriented clockwise, so we need to reverse the order
                inout_pslg.add_outline(hole_outline_points.reverse(), true);
            });
        }
    }
}

/**
 * Returns in `out_pslgs` a PSLG for each of the connected components
 * in `mask`.
 */
template<typename Scalar, typename VertexData>
void bitmask_to_component_outline_pslgs(
    std::vector<PSLG<Scalar,VertexData> >& out_pslgs,
    const BitArray& mask,
    const VertexData& default_pslg_vertex_data,
    const Scalar& outline_accuracy,
    typename optimtools::geometry_traits<Scalar,2>::Point mask_origin,
    bool is_add_holes
) {
    // Build output vector dynamically to avoid memory issues
    out_pslgs.clear();

    // Process each component one at a time to avoid memory issues
    mask.for_each_component([&](const BitArray& component_mask) {
        out_pslgs.emplace_back(default_pslg_vertex_data);
        add_bitmask_component_outline_to_pslg<Scalar,VertexData>(
            out_pslgs.back(),
            component_mask,
            outline_accuracy,
            mask_origin,
            is_add_holes
        );
    });
}

END_OPTIMTOOLS_NAMESPACE
