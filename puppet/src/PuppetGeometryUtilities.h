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

#ifndef EXTENSIONS_PUPPET_PUPPETGEOMETRYUTILITIES
#define EXTENSIONS_PUPPET_PUPPETGEOMETRYUTILITIES

#include "optimtools/common.h"

namespace extensions
{
    namespace puppet
    {
        using namespace optimtools;
        
        /**
         * A utility class that provides geometry-related operations for puppet meshes.
         */
        template<typename Scalar>
        class PuppetGeometryUtilities {
            USING_OPTIMTOOLS_MESH_TYPES(typename,Scalar,2);
            
        public:
            
            /**
             * Finds the index of the nearest or furthest point in a vector of points to a given point.
             *
             * @param points Vector of points to search through
             * @param p The target point to find the neighbor for
             * @param find_nearest If true, finds the nearest point; if false, finds the furthest point
             * @return Index of the found point, or -1 if points vector is empty
             */
            static Int find_relative_point(const PointVector& points, const Point& p, bool find_nearest) {
                Int found_vert = -1;
                double found_dist = find_nearest ? std::numeric_limits<double>::max() : 0.0;
                
                Int num_points = points.size();
                for (Int i = 0; i < num_points; ++i) {
                    Point q = points[i];
                    double d = (p - q).matrix().norm();
                    if (find_nearest ? (d < found_dist) : (d > found_dist)) {
                        found_vert = i;
                        found_dist = d;
                    }
                }
                
                return found_vert;
            }
            
            /**
             * Returns a vector containing the index of the nearest point to a given point.
             */
            static IntVector nearest_point_ind_vec(const PointVector& points, const Point& p) {
                IntVector result(1);
                result[0] = find_relative_point(points, p, true);
                return result;
            }
            
            /**
             * Returns a vector containing the index of the furthest point to a given point.
             */
            static IntVector furthest_point_ind_vec(const PointVector& points, const Point& p) {
                IntVector result(1);
                result[0] = find_relative_point(points, p, false);
                return result;
            }
                        
            /**
             * Finds the inward tangent vector at a boundary vertex of the mesh.
             * This is useful for determining a direction pointing into the mesh from the boundary,
             *
             * @param boundary_vertex_index The index of the boundary vertex.
             * @param mesh The mesh to search for the boundary and compute the tangent.
             * @return The inward tangent vector as a Point. Returns a zero vector if not computable.
             */
            static Point find_inward_boundary_tangent(Int boundary_vertex_index, const optimtools::PuppetMesh<Scalar>& mesh) {
                const PointVector& points = mesh.points();
                const TriangleVector& triangles = mesh.triangles();
                
                // Validate vertex index
                if (boundary_vertex_index < 0 || boundary_vertex_index >= points.size()) {
                    return Point::Zero();
                }
                
                // Find boundary edges that contain the target vertex
                std::vector<Int> boundary_adjacent_vertices;
                std::map<std::pair<Int, Int>, Int> edge_counts;
                
                // Count occurrences of each edge and collect boundary vertices in one pass
                for (Int tri_idx = 0; tri_idx < triangles.size(); ++tri_idx) {
                    const Triangle& tri = triangles[tri_idx];
                    for (Int i = 0; i < 3; ++i) {
                        Int v1 = tri[i];
                        Int v2 = tri[(i + 1) % 3];
                        
                        // Ensure consistent edge ordering (smaller vertex first) so that each edge is counted only once regardless of triangle orientation
                        if (v1 > v2) {
                            std::swap(v1, v2);
                        }
                        
                        auto& count = edge_counts[{v1, v2}];
                        count++;
                        
                        // If this is the first occurrence of this edge and it contains our target vertex,
                        // add the adjacent vertex to our list
                        if (count == 1) {
                            if (v1 == boundary_vertex_index) {
                                boundary_adjacent_vertices.push_back(v2);
                            } else if (v2 == boundary_vertex_index) {
                                boundary_adjacent_vertices.push_back(v1);
                            }
                        }
                        // If this is the second occurrence, remove the edge from our boundary list
                        // (since it's now an interior edge)
                        else if (count == 2) {
                            // Remove any previously added boundary vertices for this edge
                            boundary_adjacent_vertices.erase(
                                                             std::remove_if(boundary_adjacent_vertices.begin(),
                                                                            boundary_adjacent_vertices.end(),
                                                                            [v1, v2, boundary_vertex_index](Int adj_vertex) {
                                                                                return (v1 == boundary_vertex_index && adj_vertex == v2) ||
                                                                                (v2 == boundary_vertex_index && adj_vertex == v1);
                                                                            }),
                                                             boundary_adjacent_vertices.end()
                                                             );
                        }
                    }
                }
                
                // If we don't have at least 2 boundary vertices, we can't compute a proper tangent
                if (boundary_adjacent_vertices.size() < 2) {
                    return Point::Zero();
                }
                
                // Get the two boundary vertices
                Point v1 = points[boundary_adjacent_vertices[0]];
                Point v2 = points[boundary_adjacent_vertices[1]];
                Point boundary_vertex = points[boundary_vertex_index];
                
                // Compute the boundary tangent (direction along the boundary)
                Point boundary_tangent = v2 - v1;
                Scalar tangent_magnitude = boundary_tangent.matrix().norm();
                
                const Scalar epsilon = std::numeric_limits<Scalar>::epsilon();
                if (tangent_magnitude <= epsilon) {
                    return Point::Zero();
                }
                
                // Normalize the boundary tangent
                boundary_tangent /= tangent_magnitude;
                
                // Compute the inward normal by rotating the boundary tangent 90 degrees counterclockwise
                // Since triangles are CCW, the inward direction is to the left of the boundary tangent
                Point inward_normal(-boundary_tangent.y(), boundary_tangent.x());
                
                // Verify the inward direction by checking if it points toward the mesh interior
                // We can do this by checking if the inward normal points toward the centroid of adjacent triangles
                Point centroid = Point::Zero();
                Int triangle_count = 0;
                
                // Find triangles that contain the boundary vertex
                for (Int tri_idx = 0; tri_idx < triangles.size(); ++tri_idx) {
                    const Triangle& tri = triangles[tri_idx];
                    for (Int i = 0; i < 3; ++i) {
                        if (tri[i] == boundary_vertex_index) {
                            // Add the triangle's centroid
                            centroid += (points[tri[0]] + points[tri[1]] + points[tri[2]]) / 3.0;
                            triangle_count++;
                            break;
                        }
                    }
                }
                
                if (triangle_count > 0) {
                    centroid /= triangle_count;
                    
                    // Check if the inward normal points toward the centroid
                    Point to_centroid = centroid - boundary_vertex;
                    if (inward_normal.matrix().dot(to_centroid.matrix()) < 0) {
                        // Flip the normal if it's pointing away from the centroid
                        inward_normal = -inward_normal;
                    }
                }
                
                return inward_normal;
            }
            
            /**
             * Computes the average edge length of a mesh.
             */
            static Scalar get_average_edge_length(const optimtools::PuppetMesh<Scalar>& mesh) {
                const PointVector& points = mesh.points();
                const TriangleVector& triangles = mesh.triangles();
                
                // Guard against empty mesh
                if (points.size() == 0 || triangles.size() == 0) {
                    return 0.0;
                }
                
                Scalar total_length = 0.0;
                Int edge_count = 0;
                
                for (Int i = 0; i < triangles.size(); ++i) {
                    const Triangle& tri = triangles[i];
                    const Point& p1 = points[tri[0]];
                    const Point& p2 = points[tri[1]];
                    const Point& p3 = points[tri[2]];
                    
                    total_length += (p2 - p1).matrix().norm();
                    total_length += (p3 - p2).matrix().norm();
                    total_length += (p1 - p3).matrix().norm();
                    edge_count += 3;
                }
                
                return total_length / edge_count;
            }
            
            /**
             * Get the nearest point in the mesh to the given point p
             */
            static Point get_nearest_point_mesh_point(const Point& p, const optimtools::PuppetMesh<Scalar>& mesh) {
                const PointVector& points = mesh.points();
                Int nearest_index = find_relative_point(points, p, true);
                if (nearest_index >= 0) {
                    return mesh.points()[nearest_index];
                }
                return p; // Return the input point if no nearest point found
            }
            
            /**
             * Finds points within a given radius of a target point and returns the index
             * of the point with the highest depth value (z-order).
             * 
             * Used for handle placement to select the topmost vertex within some radius, with the ordering
             * defined by the point depths. We want to select the part of the mesh that's rendered on top.
             *
             * Returns the index of the point with the highest depth within radius, or -1 if none found
             */
            static Int find_highest_point_in_radius(const PointVector& points,
                                                    const Point& p,
                                                    Scalar radius,
                                                    const Eigen::Matrix<Scalar,Eigen::Dynamic,1>& point_depths) {
                Int best_point_idx = -1;
                Scalar highest_depth = std::numeric_limits<Scalar>::lowest();
                
                Int num_points = points.size();
                for (Int i = 0; i < num_points; ++i) {
                    Point q = points[i];
                    Scalar dist = (p - q).matrix().norm();
                    
                    if (dist <= radius) { // squared distance seems to work fine
                        // Point is within radius, check if it has higher depth
                        if (i < point_depths.size() && point_depths[i] > highest_depth) {
                            highest_depth = point_depths[i];
                            best_point_idx = i;
                        }
                    }
                }
                
                return best_point_idx;
            }

            /**
             * Checks if a point is inside a mesh.
             */
            static bool point_inside_mesh(const Point& p, const shared_ptr<optimtools::PuppetMesh<Scalar>> mesh) {
                Int out_triangle_ind;
                Scalar out_l1, out_l2, out_l3; // barycentric coordinates
                PuppetMeshGeometryBruteForce geometry(mesh);
                return geometry.nearest_triangle_barycentric(out_triangle_ind, out_l1, out_l2, out_l3, p);
            }

            /**
             * Computes the barycentric coordinates of a point p with respect to a triangle (a, b, c).
             * The result is stored in u, v, w.
             * Returns false if the triangle is degenerate or if the point is outside the triangle.
             */
            static bool barycentric(const Point& p, const Point& a, const Point& b, const Point& c, Scalar &u, Scalar &v, Scalar &w) {
                Point v0 = b - a, v1 = c - a, v2 = p - a;
                // Use .matrix().dot() to access dot product functionality, as Point might be an Eigen::Array
                Scalar d00 = v0.matrix().dot(v0.matrix());
                Scalar d01 = v0.matrix().dot(v1.matrix());
                Scalar d11 = v1.matrix().dot(v1.matrix());
                Scalar d20 = v2.matrix().dot(v0.matrix());
                Scalar d21 = v2.matrix().dot(v1.matrix());
                Scalar denom = d00 * d11 - d01 * d01;
                
                // Check for degenerate triangle
                constexpr Scalar degenerateEpsilon = Scalar(1e-8);
                if (std::abs(denom) < degenerateEpsilon)
                {
                    return false;
                }
                
                v = (d11 * d20 - d01 * d21) / denom;
                w = (d00 * d21 - d01 * d20) / denom;
                u = Scalar(1) - v - w;
                
                // Check if point is inside triangle (all barycentric coords non-negative)
                // Use small epsilon tolerance for floating-point edge cases
                constexpr Scalar insideEpsilon = Scalar(1e-5);
                if (u < -insideEpsilon || v < -insideEpsilon || w < -insideEpsilon)
                {
                    return false;
                }
                return true;
            }

            /**
             * Finds the vertex index from the topmost triangle containing the point p.
             * 
             * @param mesh The puppet mesh
             * @param p The point to check
             * @param point_depths Vector of point depths used to determine which triangle is on top
             * @return The index of the nearest vertex of the topmost triangle containing p, or -1 if p is not inside any triangle.
             */
            static Int find_vertex_from_topmost_triangle(
                const optimtools::PuppetMesh<Scalar>& mesh,
                const Point& p,
                const Eigen::Matrix<Scalar,Eigen::Dynamic,1>& point_depths)
            {
                const PointVector& points = mesh.points();
                const TriangleVector& triangles = mesh.triangles();
                
                Int best_tri_idx = -1;
                Scalar max_depth = std::numeric_limits<Scalar>::lowest();
                
                // Iterate over all triangles to find ones containing p
                for (Int i = 0; i < triangles.size(); ++i) {
                    const Triangle& tri = triangles[i];
                    const Point& p0 = points[tri[0]];
                    const Point& p1 = points[tri[1]];
                    const Point& p2 = points[tri[2]];
                    
                    Scalar u, v, w;
                    if (barycentric(p, p0, p1, p2, u, v, w)) {
                        // Point is inside triangle. Compute its depth.
                        Scalar current_depth = 0.0;
                        if (point_depths.size() == points.size()) {
                            // Average depth of vertices
                            current_depth = (point_depths[tri[0]] + point_depths[tri[1]] + point_depths[tri[2]]) / 3.0;
                        } else {
                            // This shouldn't happen, but if it does, let's signal that we can't compute a meaningful answer, and
                            // let the caller handle a fallback option.
                            return -1;
                        }

                        if (current_depth > max_depth) {
                            max_depth = current_depth;
                            best_tri_idx = i;
                        }
                    }
                }
                
                if (best_tri_idx != -1) {
                    // Found the topmost triangle. Return its nearest vertex.
                    const Triangle& tri = triangles[best_tri_idx];
                    const Point& p0 = points[tri[0]];
                    const Point& p1 = points[tri[1]];
                    const Point& p2 = points[tri[2]];
                    
                    Scalar d0 = (p - p0).matrix().squaredNorm();
                    Scalar d1 = (p - p1).matrix().squaredNorm();
                    Scalar d2 = (p - p2).matrix().squaredNorm();
                    
                    if (d0 <= d1 && d0 <= d2) return tri[0];
                    if (d1 <= d0 && d1 <= d2) return tri[1]; 
                    return tri[2];
                }
                
                return -1;
            }
        };
    }
}

#endif /* EXTENSIONS_PUPPET_PUPPETGEOMETRYUTILITIES */
