#pragma once

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

#include "optimtools/all_beta.h"
#include "puppet_warp/puppet_base.h"

BEGIN_OPTIMTOOLS_NAMESPACE

#define PRINT_ITERATION_TIMING 0

template<typename Scalar>
class ADMMBirdWarper {
    USING_OPTIMTOOLS_MESH_TYPES(typename,Scalar,2);
    typedef Eigen::Matrix<Scalar,Eigen::Dynamic,1> Vector;
    typedef Eigen::SparseMatrix<Scalar> SparseMatrix;
    
public:
    
#pragma mark Options

    class Options {
    public:
        Options() { } // works around a Clang bug
     
        bool is_use_l1() const { return m_is_use_l1; }
        Options& set_is_use_l1(bool b) { m_is_use_l1 = b; return *this; }
        Options is_use_l1(bool b) const { Options result = *this; result.set_is_use_l1(b); return result; }

        bool is_use_eps_p() const { return m_is_use_eps_p; }
        Options& set_is_use_eps_p(bool b) { m_is_use_eps_p = b; return *this; }
        Options is_use_eps_p(bool b) const { Options result = *this; result.set_is_use_eps_p(b); return result; }

        bool is_use_ACAP() const { return m_is_use_ACAP; }
        Options& set_is_use_ACAP(bool b) { m_is_use_ACAP = b; return *this; }
        Options is_use_ACAP(bool b) const { Options result = *this; result.set_is_use_ACAP(b); return result; }

    private:
        bool m_is_use_l1{false};
        bool m_is_use_eps_p{false};
        bool m_is_use_ACAP{false};
    };

public:
    
#pragma mark ADMMBirdWarper public methods
    ADMMBirdWarper(
        const PointVector& points,
        const TriangleVector& triangles,
        const std::vector<IntVector>& handles,
        double bird_s,
        double bird_w,
        const std::vector<IntVector>& affine_handles = {},
        Options options={}
    ) :
        m_rest_points(points),
        m_triangles(triangles),
        m_handles(handles),
        m_bird_s(bird_s),
        m_bird_w(bird_w),
        m_affine_handles(affine_handles),
        m_options(std::move(options))
    {
        init();
    }

    Eigen::AlignedBox2d rest_mesh_bounds() const {
        Eigen::AlignedBox2d result{m_rest_points[0], m_rest_points[0]};
        for(Int i = 1; i < m_points.size(); ++i) {
            result.extend(m_rest_points[i].matrix());
        }
        return result;
    }

    const PointVector& points() const {
        return m_points;
    }

    const PointVector& rest_points() const {
        return m_rest_points;
    }

    const TriangleVector& triangles() const {
        return m_triangles;
    }

    const Vector& point_w() {
        optimtools::funclets::GroupBirdLoss<DVec> bird{m_bird_w, m_bird_s};
        Int num_points = m_points.size();

        for(Int i = 0; i < num_points; ++i) {
            DVec p = m_rest_points[i];
            DVec x = m_x.row(i);
            double loss = optimtools::funclets::set_vars_eval(bird, x - p);
            m_point_w[i] = 2*loss / (m_bird_w*m_bird_s);
        }

        return m_point_w;
    }

    // Returns the vertex indices for each handle
    const std::vector<IntVector>& handles() const {
        return m_handles;
    }

    // Returns the point locations for each handle
    const std::vector<PointVector>& handle_points() const {
        return m_handle_points;
    }

    std::vector<PointVector>& handle_points() {
        return m_handle_points;
    }

    const std::vector<IntVector>& affine_handles() const {
        return m_affine_handles;
    }

    double bird_w() const {
        return m_bird_w;
    }

    void set_bird_w(double w) {
        if(w != m_bird_w) {
            m_bird_w = w;
            // init_energy(); // updates `rho`
        }
    }

    double bird_s() const {
        return m_bird_s;
    }

    void set_bird_s(double s) {
        if(s != m_bird_s) {
            m_bird_s = s;
            // init_energy(); // updates `rho`
        }
    }

    void set_max_iterations(double max_iterations) {
        m_max_iterations = static_cast<Int>(max_iterations);
    }

    void reset() {
        std::cout<<"Bird Warper reset\n";
        Int num_triangles = m_triangles.size();

        stack_points(m_x, m_rest_points);
        stack_points(m_x0, m_rest_points);
        stack_handle_points(m_xeq, m_handle_points);
        m_prev_x = m_x;

        m_z = m_x - m_x0;
        m_u.setZero();
        m_R.setZero();
        for(Int i = 0; i < num_triangles; ++i) {
            m_R.block(2*i,0,2,2).setIdentity();
        }
    }

    Int iterate(Int n=-1) {
        set_handle_constraints();
        Int max_iterations = (0 < n) ? n : m_max_iterations;

        Int num_iterations = 0;

        Int num_points = m_points.size();
        for(Int i = 0; i < max_iterations; ++i) {
            auto x =  m_x.topRows(num_points);
            m_prev_x = x;

            ++num_iterations;

            // update R
            {
                m_R.noalias() = -m_K*x;
                if(m_options.is_use_ACAP()) {
                    optimtools::internal::arap_utils<double,2>::fit_packed_similarity_transforms(m_R);
                } else {
                    optimtools::internal::arap_utils<double,2>::fit_packed_rotations(m_R);
                }
            }

            // update z
            {
                m_z = x - m_x0 + m_u;
                if(m_options.is_use_l1()) {
                    using GroupL1 = optimtools::funclets::DistanceFunction<
                        DVec,
                        optimtools::funclets::ZeroConstraint,
                        optimtools::funclets::scalar_funclets::Abs
                    >;
                    auto l1 = GroupL1{};

                    for(Int i = 0; i < num_points; ++i) {
                        DVec z = m_z.row(i);
                        auto f = (m_bird_w*m_a[i]) * l1;
                        m_z.row(i) = optimtools::funclets::set_vars_prox(f, m_rho, z);
                    }
                } else {
                    optimtools::funclets::GroupBirdLoss<DVec> bird{m_bird_w, m_bird_s};

                    for(Int i = 0; i < num_points; ++i) {
                        DVec z = m_z.row(i);
                        auto f = m_a[i] * bird;
                        m_z.row(i) = optimtools::funclets::set_vars_prox(f, m_rho, z);
                    }
                }
            }

            // update x
            {
                auto B = m_B.topRows(num_points);
                B.noalias() = m_K.transpose()*m_R;
                B -= m_rho*(m_x0 + m_z - m_u);
                if(0.0 < m_beta) {
                    B -= m_B_beta;
                }
                m_B.bottomRows(m_num_aux_vars).setZero();
                m_lcq_solver.solve_to(m_x, m_B, m_xeq);
            }

            // update u
            {
                m_u += x - m_x0 - m_z;
            }

            // convergence test
            if(n < 0) {
                if((x - m_prev_x).norm() <= m_eps) {
                    break;
                }
            }
        }

        unstack_points(m_points, m_x.topRows(num_points));
        
        return num_iterations;
    }

    void solve() {
        Int num_iterations = -1;
        
#if PRINT_ITERATION_TIMING
        double secs = optimtools::timeit([&]{
            num_iterations = iterate();
        });
#else
        num_iterations = iterate();
#endif
        
        assert(0 <= num_iterations);

#if PRINT_ITERATION_TIMING
        if(5 <= num_iterations) {
            m_iteration_timer.add(secs);
            std::cout << m_iteration_timer.value() << "    : num_iterations = " << num_iterations << std::endl;
        }
#endif
    }

    /**
     * New function. Solve for approximate distance which can be used as a proxy for depth when regions are overlapping.
     */
    void solve_laplacian_system_to(Vector& out_x, const Vector& b, const Vector& xeq) const {
        assert(b.size() == points().size());
        assert(xeq.size() == handles().size());

        if(xeq.size() > 0) {
            // Evaluate right hand side to concrete vector type. (Otherwise, I was getting a compile error)
            Vector rhs = m_lumped_mass.cwiseProduct(b);
            m_laplacian_lcq_solver.solve_to(out_x, rhs, xeq);
        } else {
            out_x.resize(points().size());
            out_x.setZero();
        }
    }
    

#pragma mark ADMMBirdWarper private methods
private:
    /**
     * Initialize the ADMM Bird Warper with mesh data and handles.
     * 
     * This method sets up the warper for deformation by:
     * 1. Initializing handle points from vertex indices to 2D coordinates
     * 2. Computing adaptive convergence threshold based on mesh scale
     * 3. Setting up ADMM variables (x, z, u, R matrices)
     * 4. Initializing the energy formulation and constraint matrices
     * 
     * The warper is ready for iteration after this initialization.
     */
    void init() {
        m_points = m_rest_points;

        // Initialize handle points by converting vertex indices to actual 2D coordinates
        m_handle_points.resize(m_handles.size());
        for(Int handle_ind = 0; handle_ind < m_handles.size(); ++handle_ind) {
            const IntVector& handle = m_handles[handle_ind];
            PointVector& handle_points = m_handle_points[handle_ind];

            // Resize handle_points to match the number of vertices controlled by this handle
            handle_points.resize(handle.size());
            for(Int i = 0; i < handle.size(); ++i) {
                // Convert vertex index to actual 2D point coordinates from the rest mesh
                // handle[i] is the vertex index, m_rest_points[handle[i]] is the 2D position
                handle_points[i] = m_rest_points[handle[i]];
            }
        }

        Int num_points = m_points.size();
        Int num_triangles = m_triangles.size();
        
        // Initialize point movement weights vector
        m_point_w.resize(num_points);
        m_point_w.setZero();

        // Calculate adaptive convergence threshold based on mesh scale
        if(m_options.is_use_eps_p()) {
            // Compute average edge length across the entire mesh
            double avg_edge_length = 0.0;
            Int num_edges = 0;
            
            // Iterate through all triangles to sum up edge lengths
            for(Int triangle_ind = 0; triangle_ind < num_triangles; ++triangle_ind) {
                const Triangle& tri = m_triangles[triangle_ind];
                // Get the 2D coordinates of the three triangle vertices
                Point p0 = m_rest_points[tri[0]];
                Point p1 = m_rest_points[tri[1]];
                Point p2 = m_rest_points[tri[2]];
                
                // Calculate Euclidean length of each triangle edge and accumulate
                avg_edge_length += (p1 - p0).matrix().norm();  // Edge from vertex 0 to 1
                avg_edge_length += (p2 - p1).matrix().norm();  // Edge from vertex 1 to 2
                avg_edge_length += (p2 - p0).matrix().norm();  // Edge from vertex 0 to 2
                num_edges += 3;  // Each triangle contributes 3 edges
            }
            
            // Compute the average edge length across the entire mesh
            avg_edge_length /= num_edges;

            // Set convergence threshold to 0.5% of average edge length
            // This ensures the algorithm stops when vertex movements become negligible
            // relative to the mesh scale, rather than using a fixed tolerance
            m_eps = 0.005 * avg_edge_length;
        }

        init_energy();
        Int num_vars = num_points + m_num_aux_vars;

        m_x.resize(num_vars, 2);
        auto x = m_x.topRows(num_points);
        stack_points(x, m_points);
        stack_points(m_x0, m_rest_points);
        m_prev_x = x;

        m_z = x - m_x0;
        m_u = PointColMatrix::Zero(num_points,2);
        m_R = PointColMatrix::Zero(2*num_triangles,2);
        for(Int i = 0; i < num_triangles; ++i) {
            m_R.block(2*i,0,2,2).setIdentity();
        }
        m_B = PointColMatrix::Zero(num_vars, 2);

        m_init_bird_s = m_bird_s;
    }

    void init_energy() {
        Int num_points = m_points.size();
        Int num_triangles = m_triangles.size();
        Int num_handles = m_handles.size();
        Int num_handle_points = 0;
        for(const IntVector& handle_inds : m_handles) {
            num_handle_points += handle_inds.size();
        }

        Int num_affine_handles = m_affine_handles.size();
        Int num_affine_handle_points = 0;
        for(const IntVector& handle_inds : m_affine_handles) {
            num_affine_handle_points += handle_inds.size();
        }
        m_num_aux_vars = 3*num_affine_handles;

        Int num_vars = num_points + m_num_aux_vars;
        Int num_cons = num_handle_points + num_affine_handle_points;

        auto bounds = this->rest_mesh_bounds();
        double bounds_scale = bounds.diagonal().cwiseAbs().maxCoeff();
        m_bird_s *= bounds_scale / 100.0;

        m_a.resize(num_points);
        m_a.setZero();
        for(Int tri_ind = 0; tri_ind < num_triangles; ++tri_ind) {
            const Triangle& tri = m_triangles[tri_ind];
            const Point& p0 = m_rest_points[tri[0]];
            const Point& p1 = m_rest_points[tri[1]];
            const Point& p2 = m_rest_points[tri[2]];

            double w = optimtools::triangle_area(p0, p1, p2) / 3.0;
            if(isnan(w)) {
                w = 0;
            }
            m_a[tri[0]] += w;
            m_a[tri[1]] += w;
            m_a[tri[2]] += w;
        }

        double rho = 0.0;
        for(Int point_ind = 0; point_ind < num_points; ++point_ind) {
            rho = std::max(rho, m_a[point_ind]*m_bird_w/m_bird_s);
        }
        m_rho = 2.0 * rho;

        m_xeq.resize(num_cons, 2);
        m_constraint_matrix = optimtools::build_matrix<SparseMatrix>(num_cons, num_vars, [&](auto& Aeq) {
            Int con_ind = 0;

            // positional constraints
            for(Int handle_ind = 0; handle_ind < num_handles; ++handle_ind) {
                const IntVector& handle_inds = m_handles[handle_ind];
                const PointVector& handle_points = m_handle_points[handle_ind];
                for(Int handle_subind = 0; handle_subind < handle_inds.size(); ++handle_subind) {
                    Int point_ind = handle_inds[handle_subind];
                    Aeq(con_ind, point_ind) += 1.0;
                    m_xeq.row(con_ind) = handle_points[handle_subind];
                    con_ind += 1;
                }
            }
            assert(con_ind == num_handle_points);

            // affine constraints
            for(Int handle_ind = 0; handle_ind < num_affine_handles; ++handle_ind) {
                const IntVector& handle_inds = m_affine_handles[handle_ind];
                optimtools::Range<3> affine_var_range(num_points+3*handle_ind);
                for(Int handle_subind = 0; handle_subind < handle_inds.size(); ++handle_subind) {
                    Int point_ind = handle_inds[handle_subind];
                    const Point& p = m_rest_points[point_ind];
                    Aeq(con_ind, affine_var_range[0]) += p.x();
                    Aeq(con_ind, affine_var_range[1]) += p.y();
                    Aeq(con_ind, affine_var_range[2]) += 1.0;
                    Aeq(con_ind, point_ind) += -1.0;
                    m_xeq.row(con_ind).setZero();
                    con_ind += 1;
                }
            }
            assert(con_ind == num_cons);
        });

        SparseMatrix A;
        build_arap_mats(A, m_K); // must be called after m_constraint_matrix is initialized

        m_lcq_solver = optimtools::LCQSolver<SparseMatrix>(A, m_constraint_matrix);
    }

    void build_arap_mats(SparseMatrix& out_A, SparseMatrix& out_K) {
        Int num_points = m_rest_points.size();
        Int num_triangles = m_triangles.size();
        Int num_vars = num_points + m_num_aux_vars;

        SparseMatrix Q;
        if(0.0 < m_beta) {
            SparseMatrix L;
            Vector M_diag;
            optimtools::cotangent_laplacian<double,2>::mesh_laplacian(L, m_rest_points, m_triangles);
            optimtools::cotangent_laplacian<double,2>::lumped_mass(M_diag, m_rest_points, m_triangles);
            M_diag = M_diag.cwiseInverse();
            Q = L * M_diag.asDiagonal() * L;
            Q *= m_beta;

            PointColMatrix x0;
            stack_points(x0, m_rest_points);
            m_B_beta = Q*x0;
        }

        // Build Laplacian solver for depth ordering
        SparseMatrix L;
        optimtools::cotangent_laplacian<double,2>::mesh_laplacian(L, m_rest_points, m_triangles);
        optimtools::cotangent_laplacian<double,2>::lumped_mass(m_lumped_mass, m_rest_points, m_triangles);
        m_laplacian_lcq_solver = optimtools::LCQSolver<SparseMatrix>(L, m_constraint_matrix);

        optimtools::build_matrices()
            .build(out_A, num_vars, num_vars)
            .build(out_K, 2*num_triangles, num_points)
        .run([&](auto& A, auto& K) {
            optimtools::mesh_element_utils<double,2>::loop_triangles(m_rest_points, m_triangles, [&](const auto& tri_info) {
                if(m_options.is_use_ACAP()) {
                    auto triq = m_arap_element_energy.triangle_quadratic_energy(tri_info);
                    A(tri_info.point_inds(), tri_info.point_inds()) += triq.A();
                    double w = optimtools::triangle_area(tri_info.p1(), tri_info.p2(), tri_info.p3());
                    K(tri_info.deformation_inds(), tri_info.point_inds()) += (1.0/std::sqrt(w)) * triq.b().transpose();
                } else {
                    auto triq = m_arap_element_energy.triangle_quadratic_energy(tri_info);
                    A(tri_info.point_inds(), tri_info.point_inds()) += triq.A();
                    K(tri_info.deformation_inds(), tri_info.point_inds()) += triq.b().transpose();
                }
            });

            optimtools::Range<> point_range(0, num_points);

            if(0.0 < m_beta) {
                A(point_range,point_range) += Q;
            }

            Eigen::VectorXd ones = Eigen::VectorXd::Ones(num_points);
            A(point_range,point_range) += m_rho * ones.asDiagonal();
        });
    }

    double triangle_rest_area(Int i) const {
        const Triangle& tri = m_triangles[i];
        return optimtools::triangle_area(
            m_rest_points[tri[0]],
            m_rest_points[tri[1]],
            m_rest_points[tri[2]]
        );
    }

    void set_handle_constraints() {
        Int num_handles = m_handles.size();

        Int con_ind = 0;
        for(Int handle_ind = 0; handle_ind < num_handles; ++handle_ind) {
            const IntVector& handle_inds = m_handles[handle_ind];
            const PointVector& handle_points = m_handle_points[handle_ind];
            for(Int handle_subind = 0; handle_subind < handle_inds.size(); ++handle_subind) {
                m_xeq.row(con_ind) = handle_points[handle_subind];
                con_ind += 1;
            }
        }
    }

    /**
     * Converts a matrix of 2D points back to a vector format.
     * Each row of the input matrix contains the (x,y) coordinates of a point.
     */
    template<typename PointColMatrixDerived>
    static void unstack_points(PointVector& out_points, const Eigen::DenseBase<PointColMatrixDerived>& mat) {
        out_points.resize(mat.rows());
        optimtools::view_as<PointColMatrix>(out_points) = mat.derived();
    }
    
    /**
     * Converts a vector of 2D points into a matrix format for optimization.
     * Each row of the output matrix contains the (x,y) coordinates of a point.
     */
    template<typename PointColMatrixDerived>
    static void stack_points(const Eigen::DenseBase<PointColMatrixDerived>& out_mat_const, const PointVector& points) {
        PointColMatrixDerived& out_mat = const_cast<PointColMatrixDerived&>(out_mat_const.derived());
        out_mat.resize(points.size(), 2);
        out_mat = optimtools::view_as<PointColMatrix>(points);
    }
    
    static void stack_handle_points(PointColMatrix& out_mat, const std::vector<PointVector>& handle_points) {
        Int num_handle_points = 0;
        for(const PointVector& points : handle_points) {
            num_handle_points += points.size();
        }
        
        out_mat.resize(num_handle_points, 2);
        
        Int i = 0;
        for(const PointVector& points : handle_points) {
            for(Int ii = 0; ii < points.size(); ++ii) {
                out_mat.row(i) = points[ii];
                i += 1;
            }
        }
    }

#pragma mark ADMMBirdWarper private members
private:
    PointVector m_points;
    PointVector m_rest_points;
    TriangleVector m_triangles;

    // Stores vertex indices for each handle
    std::vector<IntVector> m_handles;
    
    // Stores point locations for each handle
    std::vector<PointVector> m_handle_points;

    // Stores vertex indices for each affine handle
    std::vector<IntVector> m_affine_handles;
    Options m_options;

    Int m_num_aux_vars;
    PointColMatrix m_x;
    PointColMatrix m_x0;
    PointColMatrix m_xeq;

    double m_bird_w{1.0}; // bird loss parameter
    double m_bird_s{1.0}; // bird loss parameter
    double m_init_bird_s{1.0}; // used in `set_bird_s_rel`
    Vector m_a; // lumped mass
    double m_rho{0.01};
    double m_beta{0.0}; // weight on smoothing term
    PointColMatrix m_B_beta; // linear component of smoothing term

    PointColMatrix m_z;
    PointColMatrix m_u;
    PointColMatrix m_R;
    PointColMatrix m_B;

    SparseMatrix m_K;
    SparseMatrix m_constraint_matrix;
    optimtools::ARAPMeshElementEnergy<double,2> m_arap_element_energy;
    optimtools::LCQSolver<SparseMatrix> m_lcq_solver;

    // Laplacian-based depth ordering  
    Vector m_lumped_mass;
    optimtools::LCQSolver<SparseMatrix> m_laplacian_lcq_solver;

    // convergence
    PointColMatrix m_prev_x;
    Int m_max_iterations{400};
    double m_eps{0.0005};

    Vector m_point_w;
    optimtools::aggregate_statistics::Mean<double> m_iteration_timer;
};

END_OPTIMTOOLS_NAMESPACE 
