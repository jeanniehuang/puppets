#pragma once

/** \file
 * Defines the core mesh deformation solver, as used in Character Animator.
 */

#include "puppet_warp_ch/common.h"

BEGIN_OPTIMTOOLS_NAMESPACE

/**
 * Subclasses `FASTARAPMeshWarperBase` to provide the additional functionality
 * needed by Character Animator.  This includes:
 *
 *  * per-handle weights specifying how hard/soft the constraint should be
 */
template<typename Scalar, Int dimension>
class BeakerMeshWarper :
	public FASTARAPMeshWarperBase<Scalar, dimension, Eigen::SparseMatrix<Scalar>, BeakerMeshWarper<Scalar,dimension> >
	//public FASTARAPMeshWarperBase<Scalar, dimension, Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>, BeakerMeshWarper<Scalar,dimension> >
{
	// The type of the underlying matrix used to calculate the deformation.
	// May be either dense or sparse.
	//typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> ImplMatrix;
	typedef Eigen::SparseMatrix<Scalar> ImplMatrix;

	typedef BeakerMeshWarper<Scalar,dimension>                          This;
	typedef FASTARAPMeshWarperBase<Scalar, dimension, ImplMatrix, This> Super;

	USING_OPTIMTOOLS_MESH_TYPES(typename,Scalar,dimension);

	typedef typename Eigen::Matrix<Scalar, Eigen::Dynamic,1>               Vector;
	typedef typename Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Matrix;

public:
	using typename Super::HandleType;

	class WarpParams : public serialization::Serializable {
	public:
		const Scalar& handle_strength_translation_scale() const {
			return m_translation_scale;
		}

		Scalar& handle_strength_translation_scale() {
			return m_translation_scale;
		}

		const Scalar& handle_strength_rotation_scale() const {
			return m_rotation_scale;
		}

		Scalar& handle_strength_rotation_scale() {
			return m_rotation_scale;
		}

		const Scalar& handle_strength_translation_offset() const {
			return m_translation_offset;
		}

		Scalar& handle_strength_translation_offset() {
			return m_translation_offset;
		}

		const Scalar& handle_strength_rotation_offset() const {
			return m_rotation_offset;
		}

		Scalar& handle_strength_rotation_offset() {
			return m_rotation_offset;
		}

		virtual void serialize(serialization::Archive& ar) override {
			serialize_impl(ar);
		}

		template<typename Archive>
		void serialize(Archive& ar) {
			serialize_impl(ar);
		}

	protected:
		template<typename Archive>
		void serialize_impl(Archive& ar) {
			enum {
				TRANSLATION_SCALE  = 0,
				ROTATION_SCALE     = 1,
				TRANSLATION_OFFSET = 2,
				ROTATION_OFFSET    = 3
			};

			ar.object([&](auto& ar) {
				ar("translation_scale",TRANSLATION_SCALE) & m_translation_scale;
				ar("rotation_scale",ROTATION_SCALE) & m_rotation_scale;
				ar("translation_offset",TRANSLATION_OFFSET) & m_translation_offset;
				ar("rotation_offset",ROTATION_OFFSET) & m_rotation_offset;
			});
		}

	private:
		Scalar m_translation_scale{1.0};
		Scalar m_rotation_scale{0.25};
		Scalar m_translation_offset{0.0};
		Scalar m_rotation_offset{0.01};
	};

public:
	BeakerMeshWarper() { }

	template<typename ADerived, typename WDerived>
	BeakerMeshWarper(
		const PointVector& points,
		const Matrix& skinning_weights,
		const PointVector& handle_points,
		const Eigen::SparseMatrixBase<ADerived>& A,
		const Eigen::DenseBase<WDerived>& w,
		const IntVector& group_inds
	) :
		Super(points, skinning_weights, handle_points, A, w, group_inds, std::vector<HandleType>(handle_points.size(),HandleType::FIXED), internal::DoNotInit{}),
		m_handle_types(std::vector<HandleType>(handle_points.size(),HandleType::PIN))
	{
		init(
			points,
			skinning_weights,
			A,
			w
		);
	}

	template<typename ADerived, typename WDerived>
	BeakerMeshWarper(
		const PointVector& points,
		const Matrix& skinning_weights,
		const PointVector& handle_points,
		const Eigen::SparseMatrixBase<ADerived>& A,
		const Eigen::DenseBase<WDerived>& w,
		const IntVector& group_inds,
		const std::vector<HandleType>& handle_types
	) :
		Super(points, skinning_weights, handle_points, A, w, group_inds, remap_handle_types(handle_types), internal::DoNotInit{}),
		m_handle_types(handle_types)
	{
		init(
			points,
			skinning_weights,
			A,
			w
		);
	}

	template<typename ADerived, typename WDerived>
	BeakerMeshWarper(
		const PointVector& points,
		const Matrix& skinning_weights,
		const PointVector& handle_points,
		const Eigen::SparseMatrixBase<ADerived>& A,
		const Eigen::DenseBase<WDerived>& w,
		const IntVector& group_inds,
		const std::vector<HandleType>& handle_types,
		const WarpParams& warp_params
	) :
		Super(points, skinning_weights, handle_points, A, w, group_inds, remap_handle_types(handle_types), internal::DoNotInit{}),
		m_handle_types(handle_types),
		m_warp_params(warp_params)
	{
		init(
			points,
			skinning_weights,
			A,
			w
		);
	}

	virtual const HandleType& handle_type(Int handle_ind) const override {
		return m_handle_types.at(handle_ind);
	}
	
	virtual void set_handle_types(const std::vector<HandleType>& handle_types) override {
		Assert(handle_types.size() == this->num_handles()) << "you called `set_handle_types` with a vector of size " << handle_types.size() << ", but `num_handles()` is " << this->num_handles() << raise;

		// for efficiency, make this a no-op if the handles already have
		// the given types.
		bool is_change = false;
		for(Int handle_ind = 0; handle_ind < this->num_handles(); ++handle_ind) {
			if(m_handle_types.at(handle_ind) != handle_types.at(handle_ind)) {
				is_change = true;
				break;
			}
		}
		if(is_change) {
			m_handle_types = handle_types;
			if(is_remap_handle_types()) {
				this->set_handle_types_without_init(remap_handle_types(handle_types));
			} else {
				this->set_handle_types_without_init(handle_types);
			}
			this->init_constraints();
		}
	}

	void set_handle_constraint_weights(const Vector& weights) {
		Assert(weights.size() == this->num_handles()) << "you called `set_handle_constraint_weights` with a vector of size " << weights.size() << ", but `num_handles()` is " << this->num_handles() << raise;

		// for efficiency, make this a no-op if the constraint weights
		// don't change
		bool is_change = false;
		for(Int handle_ind = 0; handle_ind < this->num_handles(); ++handle_ind) {
			if(!is_almost_equal(m_handle_constraint_weights[handle_ind], weights[handle_ind])) {
				is_change = true;
				break;
			}
		}
		if(is_change) {
			m_handle_constraint_weights = weights;
			init_mesh_points_lcq();
		}
	}

	virtual void serialize(serialization::Archive& ar) override {
		serialize_impl(ar);
	}

	template<typename Archive>
	void serialize(Archive& ar) {
		serialize_impl(ar);
	}

	// Make a couple of protected methods public to give a bit more felxibility
	// to classes using this warper.  Use these with caution.
	using Super::solve_transforms;
	using Super::solve_rotations;

protected:
	virtual Scalar laplacian_regularization_weight() const override {
		return m_regularization_weight;
	}

	using Super::init_mesh_points_lcq;

	virtual void init_mesh_points_lcq(const ImplMatrix& L, const ImplMatrix& Meq) override {
		Vector constraint_weights = map_from_handle_to_constraint(m_handle_constraint_weights);

		// the following block of code attempts to scale the constraint
		// weights so that a weight of `1.0` will mean approximately the
		// same thing for each constraint.  It's just a heuristic, so
		// it won't work perfectly in all cases, but overall it helps.
		{
			// determine the default value for each constraint (in `Peq`) and
			// a scaling factor for each constraint (in `constraint_scales`)
			// based on if it's associated with the linear or the
			// translational part of an affine transform.
			// Since the linear system factors across dimension, we only need
			// to use the x-coordinates of everything (since including the
			// y-coordinates would give the same answer).
			Vector constraint_scales(Meq.rows());
			Vector constraint_offsets(Meq.rows());

			Vector Peq(Meq.rows()); 
			this->loop_handle_constraints(
				[&](Int handle_ind, Int row_ind, const Range<dimension+1>& col_range) {
					Assert(!is_remap_handle_types()) << "If `is_remap_handle_types()==true`, BeakerPuppet currently remaps all pin handles to fixed, but the pin-handle callback was somehow invoked" << raise;
					Peq[row_ind] = this->base_handle_points()[handle_ind].x();
					constraint_scales[row_ind] = m_warp_params.handle_strength_translation_scale();
					constraint_offsets[row_ind] = m_warp_params.handle_strength_translation_offset();
				}, [&](Int handle_ind, const Range<dimension> row_range, const Range<dimension+1>& col_range) {
					row_slice(Peq, row_range).setZero();
					Peq[row_range[0]] = Scalar(1);
					row_slice(constraint_scales, row_range).setConstant(m_warp_params.handle_strength_rotation_scale());
					row_slice(constraint_offsets, row_range).setConstant(m_warp_params.handle_strength_rotation_offset());
				}, [&](Int handle_ind, const Range<dimension+1> row_range, const Range<dimension+1>& col_range) {
					row_slice(Peq, row_range.template head<dimension>()).setZero();
					Peq[row_range[0]] = Scalar(1);
					Peq[row_range[dimension]] = this->base_handle_points()[handle_ind].x();

					HandleType orig_handle_type = m_handle_types.at(handle_ind); // either PIN or FIXED
					if(orig_handle_type == HandleType::FIXED) {
						row_slice(constraint_scales , row_range.template head<dimension>()).setConstant(m_warp_params.handle_strength_rotation_scale());
						row_slice(constraint_offsets, row_range.template head<dimension>()).setConstant(m_warp_params.handle_strength_rotation_offset());
						constraint_scales [row_range[dimension]] = m_warp_params.handle_strength_translation_scale();
						constraint_offsets[row_range[dimension]] = m_warp_params.handle_strength_translation_offset();
					} else if(orig_handle_type == HandleType::PIN) {
						row_slice(constraint_scales , row_range.template head<dimension>()).setZero();
						row_slice(constraint_offsets, row_range.template head<dimension>()).setConstant(m_warp_params.handle_strength_rotation_offset());
						constraint_scales [row_range[dimension]] = m_warp_params.handle_strength_translation_scale();
						constraint_offsets[row_range[dimension]] = m_warp_params.handle_strength_translation_offset();
					}  else {
						BugError() << "unexpected original handle type in FIXED constraint callback" << raise;
					}
				}
			);

			// set op the linear system we'll solve to see how much a
			// perturbation in each constraint costs.
			Vector R(dimension*this->num_groups());
			for(Int group_ind = 0; group_ind < this->num_groups(); ++group_ind) {
				auto Rblock = R.template middleRows<dimension>(dimension*group_ind);
				Rblock.setZero();
				Rblock[0] = Scalar(1);
			}
			Vector KR = -this->matrix_K()*R;

			LCQSolver<ImplMatrix> lcq(L, Meq);
			
			Vector T;
			Vector Tlam;
			//lcq.solve_to(T, Tlam, KR, Peq); assert(Tlam.squaredNorm() < 1e-3); // sanity check

			for(Int constraint_ind = 0; constraint_ind < constraint_scales.size(); ++constraint_ind) {
				// perturb the constraint and solve
				Scalar peq_i = Peq[constraint_ind];
				Peq[constraint_ind] += Scalar(1);
				lcq.solve_to(T, Tlam, KR, Peq);
				Peq[constraint_ind] = peq_i;

				// Get the Lagrange multiplier associated with the constraint,
				// this will scale the weighting of the constraint.
				assert(Tlam.size() == constraint_scales.size());
				Scalar w = fabs(Tlam[constraint_ind]);
				constraint_scales[constraint_ind] *= w;
				constraint_offsets[constraint_ind] *= w;
			}
			constraint_weights.array() *= constraint_scales.array();
			constraint_weights.array() += constraint_offsets.array();
		}

		// Now that we've rescaled the constriant weights, we can use a
		// soft-LCQ to solve for the handle transforms.
		m_Laugsolver = SoftLCQSolver<ImplMatrix>(L, Meq, constraint_weights);
	}

	virtual void solve_mesh_points_lcq(PointColMatrix& inout_T, const PointColMatrix& KR, const PointColMatrix& Peq) const override {
		PointColMatrix KRreg = KR - m_regularization_weight*inout_T;
		m_Laugsolver.solve_to(inout_T, KRreg, Peq);
	}

protected:
	template<typename ADerived, typename WDerived>
	void init(
		const PointVector& points,
		const Matrix& skinning_weights,
		const Eigen::SparseMatrixBase<ADerived>& A,
		const Eigen::DenseBase<WDerived>& w
	) {
		m_handle_constraint_weights.resize(this->num_handles());
		m_handle_constraint_weights.setConstant(std::numeric_limits<Scalar>::max());

		Super::init(
			points,
			skinning_weights,
			A,
			w
		);
	}

	template<typename Derived>
	Vector map_from_handle_to_constraint(const Eigen::DenseBase<Derived>& handle_values) const {
		Assert(handle_values.size() == this->num_handles()) << "expected " << this->num_handles() << " per-handle values, but found " << handle_values.size() << raise;
		Vector constraint_values(this->num_handle_constraints());
		this->loop_handle_constraints(
			[&](Int handle_ind, Int row_ind, const Range<dimension+1>& col_range) {
				constraint_values[row_ind] = handle_values[handle_ind];
			}, [&](Int handle_ind, const Range<dimension> row_range, const Range<dimension+1>& col_range) {
				row_slice(constraint_values, row_range).setConstant(handle_values[handle_ind]);
			}, [&](Int handle_ind, const Range<dimension+1> row_range, const Range<dimension+1>& col_range) {
				HandleType orig_handle_type = m_handle_types.at(handle_ind); // either PIN or FIXED
				if(orig_handle_type == HandleType::FIXED) {
					row_slice(constraint_values, row_range).setConstant(handle_values[handle_ind]);
				} else if(orig_handle_type == HandleType::PIN) {
					row_slice(constraint_values, row_range).setZero();
					constraint_values[row_range[dimension]] = handle_values[handle_ind];
				}  else {
					BugError() << "unexpected original handle type in FIXED constraint callback" << raise;
				}
			}
		);
		return constraint_values;
	}

	bool is_remap_handle_types() const {
		const Scalar& translation_offset = m_warp_params.handle_strength_translation_offset();
		const Scalar& rotation_offset = m_warp_params.handle_strength_rotation_offset();
		return translation_offset > Scalar(0) || rotation_offset > Scalar(0);
	}

	static std::vector<HandleType> remap_handle_types(const std::vector<HandleType>& handle_types) {
		std::vector<HandleType> remapped_handle_types(handle_types.size());
		for(Int handle_ind = 0; handle_ind < (Int)handle_types.size(); ++handle_ind) {
			HandleType handle_type = handle_types.at(handle_ind);
			if(handle_type == HandleType::PIN) {
				handle_type = HandleType::FIXED;
			}
			remapped_handle_types.at(handle_ind) = handle_type;
		}
		return remapped_handle_types;
	}

	template<typename Archive>
	void serialize_impl(Archive& ar) {
		enum {
			SUPER                     = 0,
			LCQ_SOLVER                = 1,
			HANDLE_CONSTRAINT_WEIGHTS = 2,
			HANDLE_TYPES              = 3,
			WARP_PARAMS               = 4
		};

		ar.object([&](auto& ar) {
			auto super_ar = ar("FASTARAPMeshWarperBase",SUPER);
			Super::serialize(super_ar);

			ar("lcq_solver",LCQ_SOLVER) & m_Laugsolver;
			ar("handle_constraint_weights",HANDLE_CONSTRAINT_WEIGHTS) & m_handle_constraint_weights;
			ar("handle_types",HANDLE_TYPES) & m_handle_types;
			ar("warp_params",WARP_PARAMS) & m_warp_params;
		});
	}

private:
	SoftLCQSolver<ImplMatrix> m_Laugsolver;
	Vector m_handle_constraint_weights;
	std::vector<HandleType> m_handle_types; // may differ from that stored in the superclass

	WarpParams m_warp_params;
	Scalar m_regularization_weight{1e-6};
};

END_OPTIMTOOLS_NAMESPACE
