#pragma once

/** \file
 * Defines the puppet type used within Adobe Animate.
 */

#include "puppet_warp_an/common.h"

BEGIN_OPTIMTOOLS_NAMESPACE

/**
 * Subclasses `BasicIKMeshWarper` to provide the additional functionality
 * needed by Adobe Animate.  This includes:
 *
 *  * allowing subsets of the vertices to be constrained to tranform affinely.
 */
template<typename Scalar>
class AnPuppetWarper : public BasicIKMeshWarper<Scalar, 2, AnPuppetWarper<Scalar> > {
	static constexpr Int dimension = 2;
	typedef AnPuppetWarper<Scalar> This;
	typedef BasicIKMeshWarper<Scalar, dimension, This> Super;

	USING_OPTIMTOOLS_MESH_TYPES(typename, Scalar, dimension);
	typedef typename mesh_element_utils<Scalar, dimension>::TriangleInfo TriangleInfo;
	typedef typename mesh_element_utils<Scalar, dimension>::EdgeInfo     EdgeInfo;
	typedef typename Eigen::SparseMatrix<Scalar> AMatrix;
	typedef typename Eigen::SparseMatrix<Scalar> KMatrix;

	using HandleType = typename FullMeshWarperPuppet<Scalar, This>::HandleType;
	using HandleInfo = typename FullMeshWarperPuppet<Scalar, This>::HandleInfo;

public:
	AnPuppetWarper(
		const PointVector& points,
		const TriangleVector& triangles,
		const std::vector<HandleInfo>& handle_infos,
		const Scalar& smoothing_strength=Scalar(1)
	) {
		init(points, triangles, handle_infos, smoothing_strength);
	}

	virtual ~AnPuppetWarper() { }

	// since this class allows non-point handles, we have to override the
	// the superclass function to get/set handle positions
	virtual Point handle_point(Int full_handle_ind) override {
		Int pin_handle_ind = m_full_to_pin_handle_ind_map[full_handle_ind];
		assert(pin_handle_ind >= 0);
		return Super::handle_point(pin_handle_ind);
	}

	virtual void set_handle_point(Int full_handle_ind, const Point& p) override {
		Int pin_handle_ind = m_full_to_pin_handle_ind_map[full_handle_ind];
		assert(pin_handle_ind >= 0);
		Super::set_handle_point(pin_handle_ind, p);
	}

	bool iterate() override {
		solve_rotations();
		solve_vertices();
		return true;
	}
	
	void reset_solver() {
		m_T.setZero();
		m_R = this->template stack_repeated_block<PointColMatrix>(this->num_triangles(), 1, DDMat::Identity());
		this->solve_vertices(); // needed to reset `this->variable_matrix()`
	}

protected:
	virtual Int num_extra_variables() const override {
		return (dimension+1)*m_num_affine_handles;
	}

	void solve_rotations() {
		m_R.noalias() = -(m_K.transpose() * this->point_matrix());
		internal::arap_utils<Scalar, dimension>::fit_packed_rotations(m_R);
	}

	void solve_vertices() {
		m_T.topRows(this->num_points()).noalias() = m_K * m_R;
		const auto& handle_points = this->handle_point_matrix();
		assert(handle_points.rows() == m_pin_handle_constraint_inds.size());
		for(Int pin_handle_ind = 0; pin_handle_ind < handle_points.rows(); ++pin_handle_ind) {
			Int constraint_ind = m_pin_handle_constraint_inds.at(pin_handle_ind);
			m_beq.row(constraint_ind) = handle_points.row(pin_handle_ind);
		}
		m_lcq_solver.solve_to(this->variable_matrix(), m_T, m_beq);
	}

	void init(
		const PointVector& points,
		const TriangleVector& triangles,
		const std::vector<HandleInfo>& handle_infos,
		const Scalar& smoothing_strength
	) {
		// Parse `handle_infos` into a format that'll be easier to use by
		// the other parts of this class.
		std::vector<Int> pin_handle_verts;
		std::vector<IntVector> affine_handle_verts;

		m_num_pin_handles = 0;
		m_num_affine_handles = 0;
		m_full_to_pin_handle_ind_map = IntVector::Constant(handle_infos.size(), -1);
		m_pin_handle_constraint_inds.clear();
		m_affine_handle_constraint_ranges.clear();

		{
			Int constraint_ind = 0;
			for(Int handle_ind = 0; handle_ind < int_cast<Int>(handle_infos.size()); ++handle_ind) {
				const HandleInfo& handle_info = handle_infos.at(handle_ind);
				const IntVector& verts = handle_info.verts();
				if(handle_info.handle_type() == HandleType::PIN) {
					Assert(verts.size() == 1) << "`AnPuppetWarper` PIN handles must have exactly one vertex" << raise;
					pin_handle_verts.push_back(verts[0]);
					m_full_to_pin_handle_ind_map[handle_ind] = m_num_pin_handles;
					m_pin_handle_constraint_inds.push_back(constraint_ind);
					constraint_ind += 1;
					m_num_pin_handles += 1;
				} else if(handle_info.handle_type() == HandleType::CONSTRAINT) {
					Assert(verts.size() > 1) << "`AnPuppetWarper` CONSTRAINT handles must have more than one vertex" << raise;
					affine_handle_verts.push_back(verts);
					m_affine_handle_constraint_ranges.emplace_back(constraint_ind, verts.size());
					constraint_ind += verts.size();
					m_num_affine_handles += 1;
				} else {
					Error() << "unrecognized handle type passed to `AnPuppetWarper`" << raise;
				}
			}
			m_num_constraints = constraint_ind;
		}

		// Determine the bounding box of the puppet.  The area of this box
		// will be used to scale any regularization weights.
		assert(points.size() > 0);
		Eigen::AlignedBox<Scalar,dimension> bbox(points[0].matrix(), points[0].matrix());
		for(Int point_ind = 1; point_ind < points.size(); ++point_ind) {
			bbox.extend(points[point_ind].matrix());
		}
		Scalar bbox_area = bbox.volume();

		// The superclass constructor takes the point-pin handles, so we
		// extract them here.
		std::vector<Int> point_handle_verts_vec;
		for(const HandleInfo& handle_info : handle_infos) {
			const IntVector& verts = handle_info.verts();
			if(verts.size() == 1 && handle_info.handle_type() == HandleType::PIN) {
				point_handle_verts_vec.push_back(verts[0]);
			}
		}
		IntVector point_handle_verts(point_handle_verts_vec.size());
		for(Int point_handle_ind = 0; point_handle_ind < point_handle_verts.size(); ++point_handle_ind) {
			point_handle_verts[point_handle_ind] = point_handle_verts_vec.at(point_handle_ind);
		}

		// Initialize the superclass
		Super::init(points, triangles, point_handle_verts);

		Int num_points = points.size();
		Int num_triangles = triangles.size();
		Int num_variables = this->num_variables();
		
		// we all an infinite smoothing strength, so we parse the
		// `smoothing_strength` parameter into a weight for the ARAP and
		// non-arap components.
		Scalar arap_weight(1);
		Scalar smoothing_weight = smoothing_strength;
		if(smoothing_strength >= std::numeric_limits<Scalar>::max()) {
			arap_weight = Scalar(0);
			smoothing_weight = Scalar(1);
		} else if(smoothing_strength > Scalar(1)) {
			// this should be unnecessary except for huge smoothing strengths,
			// but it'll make the values in our matrices look nicer, so why not?
			arap_weight = Scalar(1) / smoothing_strength;
			smoothing_weight = Scalar(1);
		}

		auto extra_var_range = this->extra_variable_subrange();

		AMatrix A;
		{
			build_matrices()
				.build(A, num_variables, num_variables, 9 * num_triangles)
				.build(m_K, num_points, dimension*num_triangles, 3 * dimension*num_triangles)
				.build(m_T, num_variables, dimension)
				.run(
			[&](auto& A, auto& K, auto& T) {
				mesh_element_utils<Scalar, dimension>::loop_triangles(points, triangles, [&](const TriangleInfo& tri_info) {
					auto triq = arap_weight * m_element_energy.triangle_quadratic_energy(tri_info);
					A(tri_info.point_inds(), tri_info.point_inds()) += triq.A();
					K(tri_info.point_inds(), tri_info.deformation_inds()) += triq.b();
				});
				mesh_element_utils<Scalar, dimension>::loop_interior_edges(points, triangles, [&](const EdgeInfo& edge_info) {
					const TriangleInfo& tri_info1 = edge_info.triangle_info1();
					const TriangleInfo& tri_info2 = edge_info.triangle_info2();
					auto edgeq = smoothing_weight * m_edge_energy.edge_quadratic_energy(edge_info);
				
					Eigen::Array<Int,3,1> inds1, inds2;
					inds1.template head<3>() = tri_info1.point_inds();
					inds2.template head<3>() = tri_info2.point_inds();
					A(inds1, inds1) += edgeq.A11();
					A(inds1, inds2) += edgeq.A12();
					A(inds2, inds1) += edgeq.A21();
					A(inds2, inds2) += edgeq.A22();
				});

				// add a small regularization energy on affine-vars
				for(Int affine_handle_ind = 0; affine_handle_ind < m_num_affine_handles; ++affine_handle_ind) {
					Range<dimension+1> affine_var_range((dimension+1)*affine_handle_ind);
					affine_var_range = extra_var_range[affine_var_range];
					auto I = Eigen::Matrix<Scalar,dimension+1,dimension+1>::Identity();
					A(affine_var_range,affine_var_range) += bbox_area*m_affine_regularization_weight*I;

					T(affine_var_range.template head<dimension>(),RangeAll{}) += -bbox_area*m_affine_regularization_weight*DDMat::Identity();
				}
			});
		}

		AMatrix Aeq;
		build_matrices()
			.build(Aeq, m_num_constraints, num_variables)
			.build(m_beq, m_num_constraints, dimension)
			.run(
		[&](auto& Aeq, auto& beq) {
			// add constraints for pin handles
			for(Int pin_handle_ind = 0; pin_handle_ind < m_num_pin_handles; ++pin_handle_ind) {
				Int constraint_ind = m_pin_handle_constraint_inds.at(pin_handle_ind);
				Int vert = pin_handle_verts.at(pin_handle_ind);
				Aeq(constraint_ind, vert) += Scalar(1);
			}

			// add constraints for affine handles
			for(Int affine_handle_ind = 0; affine_handle_ind < m_num_affine_handles; ++affine_handle_ind) {
				Range<dimension+1> affine_var_range((dimension+1)*affine_handle_ind);
				affine_var_range = extra_var_range[affine_var_range];
				Range<> constraint_range = m_affine_handle_constraint_ranges.at(affine_handle_ind);
				const IntVector& handle_verts = affine_handle_verts.at(affine_handle_ind);
				assert(constraint_range.size() == handle_verts.size());

				for(Int vert_ind = 0; vert_ind < handle_verts.size(); ++vert_ind) {
					Int vert = handle_verts[vert_ind];
					const Point& base_point = points[vert];
					Aeq(constraint_range[vert_ind], affine_var_range[0]) += base_point[0];
					Aeq(constraint_range[vert_ind], affine_var_range[1]) += base_point[1];
					Aeq(constraint_range[vert_ind], affine_var_range[2]) += Scalar(1);
					Aeq(constraint_range[vert_ind], vert) += -Scalar(1);
				}
			}
		});

		m_lcq_solver = LCQSolver<AMatrix>(A, Aeq);

		m_R = this->template stack_repeated_block<PointColMatrix>(num_triangles, 1, DDMat::Identity());
	}

private:
	ARAPMeshElementEnergy<Scalar, dimension> m_element_energy;
	DeformationSmoothingMeshElementEnergy<Scalar, dimension> m_edge_energy;

	Scalar m_affine_regularization_weight{1e-5};
	LCQSolver<AMatrix> m_lcq_solver;
	PointColMatrix m_beq;
	KMatrix m_K;
	PointColMatrix m_T;
	PointColMatrix m_R;

	Int m_num_constraints;
	Int m_num_pin_handles;
	Int m_num_affine_handles;
	IntVector m_full_to_pin_handle_ind_map;
	std::vector<Int> m_pin_handle_constraint_inds;
	std::vector<Range<> > m_affine_handle_constraint_ranges;
};

END_OPTIMTOOLS_NAMESPACE
