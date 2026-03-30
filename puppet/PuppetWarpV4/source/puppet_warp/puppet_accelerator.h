#pragma once

/** \file
* Defines geoemtric query acceleration structures over puppet warp meshes.

 * Adobe patent application tracking P7442-US
* Title: "Generating A Triangle Mesh for An Image Represented by Curves"
* Inventors: Vineet Batra, Matthew Fisher, Kevin Wampler, Danny Kaufman, Ankit Phogat
*/


#include "optimtools/all.h"

BEGIN_OPTIMTOOLS_NAMESPACE

/**
* Abstract class defining interface for performing geometry-related queries on a PuppetMesh.
*/
template<typename Scalar>
class PuppetMeshGeometry {
    USING_OPTIMTOOLS_MESH_TYPES(typename, Scalar, 2);

public:
    virtual ~PuppetMeshGeometry() {}
    
    /**
    * Finds the triangle in the mesh closest to `p` and returns the index
    * of the triangle along with the barycentric coordinates of `p` within
    * the triangle.  Returns `true` if `p` lies inside of the mesh, and
    * `false` if `p` lies outside the mesh (in which case at least one
    * barycentric coordinate will be negative).
    */
    virtual bool nearest_triangle_barycentric(
        Int& out_triangle_ind,
        Scalar& out_barycentric_coord_1,
        Scalar& out_barycentric_coord_2,
        Scalar& out_barycentric_coord_3,
        const Point& p
    ) = 0;
    
    virtual bool nearest_triangle_barycentric(
        Int& out_triangle_ind,
        Int& out_puppet_index,
        Scalar& out_barycentric_coord_1,
        Scalar& out_barycentric_coord_2,
        Scalar& out_barycentric_coord_3,
        const Point& p
    ) = 0;


    /**
    * Tests the performance of brute-force vs. BVH for the given mesh.
    */

//    static void test_accelerator_perf(const PuppetMesh<Scalar> &mesh) {
//        cout << "Testing accelerator performance" << endl;
//        const int test_count = 10000;
//
//        auto geometry_bf = make_shared<PuppetMeshGeometryBruteForce<Scalar> >(make_shared<PuppetMesh<Scalar> >(mesh));
//        auto geometry_bvh = make_shared<PuppetMeshGeometryBVH<Scalar> >(make_shared<PuppetMesh<Scalar> >(mesh));
//
//        std::chrono::time_point<std::chrono::system_clock> startA, endA, startB, endB, startC, endC;
//        startA = std::chrono::system_clock::now();
//
//        auto get_tri_centroid = [&](int tri_index) {
//            const Triangle& tri = mesh.triangles()[tri_index];
//            const Point& p1 = mesh.points()[tri[0]];
//            const Point& p2 = mesh.points()[tri[1]];
//            const Point& p3 = mesh.points()[tri[2]];
//            const Point p = (p1 + p2 + p3) / 3.0f;
//            return p;
//        };
//
//        // run brute force interior queries
//        for (int i = 0; i < test_count; i++) {
//            const int triangle_ind_target = rand() % mesh.triangles().size();
//            const Point p = get_tri_centroid(triangle_ind_target);
//
//            Int triangle_ind;
//            Scalar l1, l2, l3; // barycentric coordinates
//            geometry_bf->nearest_triangle_barycentric(triangle_ind, l1, l2, l3, p);
//            if (triangle_ind != triangle_ind_target)
//                cout << "triangle intersection brute force failure" << endl;
//        }
//
//        endA = std::chrono::system_clock::now();
//
//        startB = std::chrono::system_clock::now();
//        geometry_bvh->build_accelerator();
//        endB = std::chrono::system_clock::now();
//
//        startC = std::chrono::system_clock::now();
//        
//        // run bvh interior queries
//        for (int i = 0; i < test_count; i++) {
//            const int triangle_ind_target = rand() % mesh.triangles().size();
//            const Point p = get_tri_centroid(triangle_ind_target);
//
//            Int triangle_ind;
//            Scalar l1, l2, l3; // barycentric coordinates
//            geometry_bvh->nearest_triangle_barycentric(triangle_ind, l1, l2, l3, p);
//            if (triangle_ind != triangle_ind_target)
//                cout << "triangle intersection BVH failure" << endl;
//        }
//        endC = std::chrono::system_clock::now();
//
//        std::chrono::duration<double> elapsed_secondsA = endA - startA;
//        std::chrono::duration<double> elapsed_secondsB = endB - startB;
//        std::chrono::duration<double> elapsed_secondsC = endC - startC;
//
//        cout << "bf query:         " << elapsed_secondsA.count() << "s" << endl;
//        cout << "bvh construction: " << elapsed_secondsB.count() << "s" << endl;
//        cout << "bvh query:        " << elapsed_secondsC.count() << "s" << endl;
//    }
};

/**
* Brute-force PuppetMesh accelerator.
*/
template<typename Scalar>
class PuppetMeshGeometryBruteForce : public PuppetMeshGeometry<Scalar> {
    USING_OPTIMTOOLS_MESH_TYPES(typename, Scalar, 2);

public:
    PuppetMeshGeometryBruteForce(const std::vector<shared_ptr<PuppetMesh<Scalar>>>& meshes) :
        m_meshes(meshes)
    {
        size_t pointCount = 0, triangleCount = 0;
        for (const auto& mesh : m_meshes)
        {
            pointCount += mesh->points().size();
            triangleCount += mesh->triangles().size();
        }
        PointVector points(pointCount);
        TriangleVector triangles(triangleCount);
        for (size_t ii = 0, pts_counter = 0, tri_counter = 0; ii < m_meshes.size(); ii++)
        {
            const auto& mesh = m_meshes[ii ];
            const auto& pts = mesh->points();
            const auto& tris = mesh->triangles();
            for (size_t j = 0; j < tris.size(); j++)
            {
                triangles[tri_counter++] = tris[j] + pts_counter;
            }
            for (size_t j = 0; j < pts.size(); j++)
            {
                points[pts_counter++] = pts[j];
            }
        }
        
        m_points = points;
        m_triangles = triangles;
    }
    
    PuppetMeshGeometryBruteForce(const shared_ptr<PuppetMesh<Scalar> >& mesh) :
        PuppetMeshGeometryBruteForce( std::vector<shared_ptr<PuppetMesh<Scalar>>>( {mesh} ) )
    {
    }

    bool nearest_triangle_barycentric(
        Int& out_triangle_ind,
        Int& out_puppet_index,
        Scalar& out_barycentric_coord_1,
        Scalar& out_barycentric_coord_2,
        Scalar& out_barycentric_coord_3,
        const Point& p
    ) override {
        bool is_in_mesh = this->to_triangle_barycentric_in_mesh(
            out_triangle_ind,
            out_barycentric_coord_1,
            out_barycentric_coord_2,
            out_barycentric_coord_3,
            p,
            m_points,
            m_triangles
        );
        if (!is_in_mesh) {
            to_nearest_triangle_edge_barycentric(
                out_triangle_ind,
                out_barycentric_coord_1,
                out_barycentric_coord_2,
                out_barycentric_coord_3,
                p,
                m_points,
                m_triangles
            );
        }
        return is_in_mesh;
    }
    
    bool nearest_triangle_barycentric(
        Int& out_triangle_ind,
        Scalar& out_barycentric_coord_1,
        Scalar& out_barycentric_coord_2,
        Scalar& out_barycentric_coord_3,
        const Point& p
    ) override {
        Int dummy_puppet_index = -1;
        return nearest_triangle_barycentric(out_triangle_ind,
                                            dummy_puppet_index,
                                            out_barycentric_coord_1,
                                            out_barycentric_coord_2,
                                            out_barycentric_coord_3,
                                            p);
    }


protected:
    /**
    * Returns in `out_triangle_ind` the triangle in the mesh defined by the
    * `points` and `triangles` parameters which contains `p`.  Returns `true`
    * if there exists a triangle containing `p`, and `false` otherwise.  If
    * such a triangle is found, `out_barycentric_coord_{1,2,3}` are set to
    * the barycentric coordinates of `p` withint that triangle.
    */
    template<typename PointVectorDerived, typename TriangleVectorDerived>
    static bool to_triangle_barycentric_in_mesh(
        Int& out_triangle_ind,
        Scalar& out_barycentric_coord_1,
        Scalar& out_barycentric_coord_2,
        Scalar& out_barycentric_coord_3,
        const Point& p,
        const Eigen::DenseBase<PointVectorDerived>& points,
        const Eigen::DenseBase<TriangleVectorDerived>& triangles
    ) {
        static const Scalar epsilon = sqrt(std::numeric_limits<Scalar>::epsilon());

        out_triangle_ind = -1;
        for (Int triangle_ind = 0; triangle_ind < triangles.size(); ++triangle_ind) {
            const Triangle& tri = triangles[triangle_ind];
            const Point& p1 = points[tri[0]];
            const Point& p2 = points[tri[1]];
            const Point& p3 = points[tri[2]];
            Scalar l1, l2, l3; // barycentric coordinates
            to_triangle_barycentric(l1, l2, l3, p, p1, p2, p3);

            bool is_in_triangle = (-epsilon <= l1) && (l1 <= (1 + epsilon))
                && (-epsilon <= l2) && (l2 <= (1 + epsilon))
                && (-epsilon <= l3) && (l3 <= (1 + epsilon));
            if (is_in_triangle) {
                out_triangle_ind = triangle_ind;
                out_barycentric_coord_1 = l1;
                out_barycentric_coord_2 = l2;
                out_barycentric_coord_3 = l3;
                return true;
            }
        }
        return false;
    }

    /**
    * Returns in `out_triangle_ind` the triangle in the mesh defined by the
    * `points` and `triangles` parameters for which `p` has a minimal
    * distance to one of the triangle's edges.  The
    * `out_barycentric_coord_{1,2,3}` parameters are set to the barycentric
    * coordinates of `p` within that triangle.  It is expected that `p` does
    * not lie within any of the triangles in the mesh (although the method
    * will probably work anyway if it does).
    */
    template<typename PointVectorDerived, typename TriangleVectorDerived>
    static void to_nearest_triangle_edge_barycentric(
        Int& out_triangle_ind,
        Scalar& out_barycentric_coord_1,
        Scalar& out_barycentric_coord_2,
        Scalar& out_barycentric_coord_3,
        const Point& p,
        const Eigen::DenseBase<PointVectorDerived>& points,
        const Eigen::DenseBase<TriangleVectorDerived>& triangles
    ) {
        out_triangle_ind = -1;
        Scalar min_dist = std::numeric_limits<Scalar>::max();
        for (Int triangle_ind = 0; triangle_ind < triangles.size(); ++triangle_ind) {
            const Triangle& tri = triangles[triangle_ind];
            const Point& p1 = points[tri[0]];
            const Point& p2 = points[tri[1]];
            const Point& p3 = points[tri[2]];
            Scalar d1 = point_segment_distance<Scalar, 2>(p, p1, p2);
            Scalar d2 = point_segment_distance<Scalar, 2>(p, p2, p3);
            Scalar d3 = point_segment_distance<Scalar, 2>(p, p3, p1);
            Scalar d = std::min(d1, std::min(d2, d3));
            if (d < min_dist) {
                min_dist = d;
                out_triangle_ind = triangle_ind;
                to_triangle_barycentric(
                    out_barycentric_coord_1,
                    out_barycentric_coord_2,
                    out_barycentric_coord_3,
                    p, p1, p2, p3
                );
            }
        }
    }

private:
    std::vector<shared_ptr<PuppetMesh<Scalar>>> m_meshes;
    PointVector m_points;
    TriangleVector m_triangles;
};

/**
* Bounding volume hierarchy node. For now, interior and leaf nodes are in the same
* class, but they could also be broken into two classes.
*/
template<typename Scalar>
struct PuppetMeshBVHNode {
    USING_OPTIMTOOLS_MESH_TYPES(typename, Scalar, 2);
    // TODO: add bboxes to USING_OPTIMTOOLS_MESH_TYPES
    typedef typename Eigen::AlignedBox <Scalar, 2> BBox;

    static const int leaf_tri_threshold = 32;

    PuppetMeshBVHNode() {
        left_child_index = -1;
        right_child_index = -1;
        start_tri_index = -1;
        tri_count = 0;
    }

    /**
    * Bounding volume hierarchy PuppetMesh accelerator.
    */
    void construct(
        const std::vector<BBox> &tri_bboxes,
        const std::vector<int> &node_tris,
        std::vector<PuppetMeshBVHNode<Scalar> > &nodes,
        std::vector<int> &tri_indices
    ) {
        // compute bbox
        for (int tri_ind : node_tris)
            bbox.extend(tri_bboxes[tri_ind]);

        // make this a leaf node if there are sufficiently few triangles
        if (tri_bboxes.size() <= leaf_tri_threshold) {
            make_leaf_node(node_tris, tri_indices);
            return;
        }
        
        // split on the longest dimension
        const int split_axis = (bbox.sizes()[0] > bbox.sizes()[1]) ? 0 : 1;

        // TODO: use a better splitting point
        const Scalar split_value = bbox.center()[split_axis];

        // accumulate triangles in left and right split
        std::vector<int> left_tris, right_tris;
        left_tris.reserve(node_tris.size());
        right_tris.reserve(node_tris.size());
        for (int tri_ind : node_tris)
        {
            const BBox &bbox = tri_bboxes[tri_ind];
                
            if (bbox.min()[split_axis] <= split_value)
                left_tris.push_back(tri_ind);

            if (bbox.max()[split_axis] >= split_value)
                right_tris.push_back(tri_ind);
        }

        // if we fail to find a valid split, make this a leaf node
        if (left_tris.size() == node_tris.size() ||
            right_tris.size() == node_tris.size() ||
            left_tris.size() == 0 ||
            right_tris.size() == 0) {
            make_leaf_node(node_tris, tri_indices);
            return;
        }

        const int left_child_index_store = nodes.size();
        const int right_child_index_store = nodes.size() + 1;

        left_child_index = left_child_index_store;
        right_child_index = right_child_index_store;

        // note that once we resize nodes, we potentially destroy our this pointer,
        // so we can no longer safely modify or reference member variables.
        nodes.push_back(PuppetMeshBVHNode<Scalar>());
        nodes.push_back(PuppetMeshBVHNode<Scalar>());

        nodes[left_child_index_store].construct(tri_bboxes, left_tris, nodes, tri_indices);
        nodes[right_child_index_store].construct(tri_bboxes, right_tris, nodes, tri_indices);
    }

    template<typename PointVectorDerived, typename TriangleVectorDerived>
    bool to_triangle_barycentric_in_mesh(
        Int& out_triangle_ind,
        Scalar& out_barycentric_coord_1,
        Scalar& out_barycentric_coord_2,
        Scalar& out_barycentric_coord_3,
        const Point& p,
        const Eigen::DenseBase<PointVectorDerived>& points,
        const Eigen::DenseBase<TriangleVectorDerived>& triangles,
        const std::vector<PuppetMeshBVHNode<Scalar> > &nodes,
        const std::vector<int> &tri_indices
    ) const {
        if (!bbox.contains(p.matrix())) {
            return false;
        }

        if (is_leaf()) {
            // test all triangles in the leaf node
            static const Scalar epsilon = sqrt(std::numeric_limits<Scalar>::epsilon());

            out_triangle_ind = -1;
            for (int triangle_ind_ind = start_tri_index; triangle_ind_ind < start_tri_index + tri_count; ++triangle_ind_ind) {
                const Triangle& tri = triangles[tri_indices[triangle_ind_ind]];
                const Point& p1 = points[tri[0]];
                const Point& p2 = points[tri[1]];
                const Point& p3 = points[tri[2]];
                
                // TODO: do bbox test first
                Scalar l1, l2, l3; // barycentric coordinates
                to_triangle_barycentric(l1, l2, l3, p, p1, p2, p3);

                bool is_in_triangle = (-epsilon <= l1) && (l1 <= (1 + epsilon))
                    && (-epsilon <= l2) && (l2 <= (1 + epsilon))
                    && (-epsilon <= l3) && (l3 <= (1 + epsilon));
                if (is_in_triangle) {
                    out_triangle_ind = tri_indices[triangle_ind_ind];
                    out_barycentric_coord_1 = l1;
                    out_barycentric_coord_2 = l2;
                    out_barycentric_coord_3 = l3;
                    return true;
                }
            }
            return false;
        }

        const PuppetMeshBVHNode<Scalar>& left_child = nodes[left_child_index];
        const PuppetMeshBVHNode<Scalar>& right_child = nodes[right_child_index];

        bool is_in_mesh = left_child.to_triangle_barycentric_in_mesh(
            out_triangle_ind, out_barycentric_coord_1, out_barycentric_coord_2, out_barycentric_coord_3, p, points, triangles, nodes, tri_indices);
        if (is_in_mesh) return true;

        return right_child.to_triangle_barycentric_in_mesh(
            out_triangle_ind, out_barycentric_coord_1, out_barycentric_coord_2, out_barycentric_coord_3, p, points, triangles, nodes, tri_indices);
    }

    bool is_leaf() const {
        return (tri_count > 0);
    }

    // bounding box of all triangles including and beneath this node
    BBox bbox;

    // indices of the child nodes in the global node list (interior only)
    unsigned int left_child_index, right_child_index;

    // indices into the global triangle index list (leaf only)
    unsigned int start_tri_index, tri_count;

private:
    /**
    * Initalizes this node as a leaf node.
    */
    void make_leaf_node(
        const std::vector<int> &node_tris,
        std::vector<int> &tri_indices)
    {
        start_tri_index = (unsigned int)tri_indices.size();
        tri_count = (unsigned int)node_tris.size();
        for (int tri_ind : node_tris)
            tri_indices.push_back(tri_ind);
    }
};

/**
* Bounding volume hierarchy PuppetMesh accelerator.
*/
//https://eigen.tuxfamily.org/dox/classEigen_1_1AlignedBox.html
template<typename Scalar>
class PuppetMeshGeometryBVH : public PuppetMeshGeometry<Scalar> {
    USING_OPTIMTOOLS_MESH_TYPES(typename, Scalar, 2);
    typedef typename Eigen::AlignedBox <Scalar, 2> BBox;

public:
    PuppetMeshGeometryBVH(const std::vector<shared_ptr<PuppetMesh<Scalar>>>& meshes) :
        m_meshes(meshes),
        m_brute_force_fallback(meshes)
    { }

    /**
    * check if the acceleration structure has been constructed.
    */
    bool accelerator_built() const
    {
        return m_nodes.size() > 0;
    }

    /**
    * builds the acceleration structure. This will also be called automatically
    * when a query is called.
    */
    void build_accelerator()
    {
        if (accelerator_built()) return;

        std::vector<BBox> tri_bboxes;
        std::vector<int> root_tris;
        m_triangleCount = 0;
        m_pointCount = 0;
        size_t root_tri_counter = 0;
        for (const auto& mesh : m_meshes)
        {
            const auto &points = mesh->points();
            const auto &triangles = mesh->triangles();
            m_triangleCount += triangles.size();
            m_pointCount += points.size();
            // compute all triangle bounding boxes
            for (Int triangle_ind = 0; triangle_ind < triangles.size(); ++triangle_ind) {
                const Triangle& tri = triangles[triangle_ind];
                const Point& p1 = points[tri[0]];
                const Point& p2 = points[tri[1]];
                const Point& p3 = points[tri[2]];
                BBox box;
                box.extend(p1.matrix());
                box.extend(p2.matrix());
                box.extend(p3.matrix());
                tri_bboxes.push_back(box);
            }

            for (int i = 0; i < triangles.size(); i++)
                root_tris.push_back(root_tri_counter++);
        }
        
        m_points = std::make_unique<PointVector>(m_pointCount);
        m_triangles = std::make_unique<TriangleVector>(m_triangleCount);
        m_puppet_tri_count.clear();
        for (size_t ii = 0, pts_counter = 0, tri_counter = 0; ii < m_meshes.size(); ii++)
        {
            const auto& mesh = m_meshes[ii];
            const auto& pts = mesh->points();
            const auto& tris = mesh->triangles();
            
            m_puppet_tri_count.push_back(mesh->triangles().size());
            for (size_t j = 0; j < tris.size(); j++)
            {
                (*m_triangles)[tri_counter++] = tris[j] + pts_counter;
            }
            for (size_t j = 0; j < pts.size(); j++)
            {
                (*m_points)[pts_counter++] = pts[j];
            }
        }

        m_nodes.reserve(m_triangleCount / PuppetMeshBVHNode<Scalar>::leaf_tri_threshold);
        m_nodes.push_back(PuppetMeshBVHNode<Scalar>());
        m_nodes[0].construct(tri_bboxes, root_tris, m_nodes, m_tri_indices);
    }
    
    bool nearest_triangle_barycentric(
        Int& out_triangle_ind,
        Int& out_puppet_index,
        Scalar& out_barycentric_coord_1,
        Scalar& out_barycentric_coord_2,
        Scalar& out_barycentric_coord_3,
        const Point& p
    ) override {
        build_accelerator();
        
        const PuppetMeshBVHNode<Scalar> &root = m_nodes[0];
        bool is_in_mesh = root.to_triangle_barycentric_in_mesh(
            out_triangle_ind, out_barycentric_coord_1, out_barycentric_coord_2, out_barycentric_coord_3,
            p, *m_points, *m_triangles, m_nodes, m_tri_indices);
        if (!is_in_mesh)
        {
            // if not in mesh, do a brute-force search over all triangles to make sure we find the closest.
            // approximate methods could be used here but they would likely have unpredictable behavior.
            is_in_mesh = m_brute_force_fallback.nearest_triangle_barycentric(out_triangle_ind,
                                                                             out_puppet_index,
                                                                             out_barycentric_coord_1,
                                                                             out_barycentric_coord_2,
                                                                             out_barycentric_coord_3,
                                                                             p);
        }
        
        for (size_t ii = 0, triangles_till_now = 0; ii < m_puppet_tri_count.size(); ii++)
        {
            if (out_triangle_ind < m_puppet_tri_count[ii] + triangles_till_now )
            {
                out_puppet_index = ii;
                out_triangle_ind -= triangles_till_now;
                break;
            }
            triangles_till_now += m_puppet_tri_count[ii];
        }
        
        return is_in_mesh;
    }
    
    bool nearest_triangle_barycentric(
        Int& out_triangle_ind,
        Scalar& out_barycentric_coord_1,
        Scalar& out_barycentric_coord_2,
        Scalar& out_barycentric_coord_3,
        const Point& p
    ) override {
        Int dummy_puppet_index = -1;
        return nearest_triangle_barycentric(out_triangle_ind,
                                            dummy_puppet_index,
                                            out_barycentric_coord_1,
                                            out_barycentric_coord_2,
                                            out_barycentric_coord_3,
                                            p);
    }

private:
    std::vector<shared_ptr<PuppetMesh<Scalar>>> m_meshes;
    PuppetMeshGeometryBruteForce<Scalar> m_brute_force_fallback;
    size_t m_triangleCount = 0;
    size_t m_pointCount = 0;
    std::vector <int> m_puppet_tri_count;
    std::unique_ptr<PointVector> m_points;
    std::unique_ptr<TriangleVector> m_triangles;

    // list of all nodes. Nodes store indexes into this list to refrence children.
    // node 0 is the root. Mutable because the accelerator is treated as a cache.
    mutable std::vector<PuppetMeshBVHNode<Scalar> > m_nodes;

    // list of all triangles. Nodes store indices into this list to reference mesh triangles.
    // Mutable because the accelerator is treated as a cache.
    mutable std::vector<int> m_tri_indices;
};

END_OPTIMTOOLS_NAMESPACE
