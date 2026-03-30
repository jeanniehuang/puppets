#pragma once

/** \file
 * Defines the puppet type used within Character Animator.
 */

#include "puppet_warp_ch/common.h"
#include "puppet_warp_ch/beaker_warper.h"

BEGIN_OPTIMTOOLS_NAMESPACE

/**
 * Subclasses `LBSPuppetHandle` to add extra fields used by Character Animator.
 */
template<typename Scalar, Int dimension>
class BeakerPuppetHandle : public LBSPuppetHandle<Scalar,dimension> {
	typedef LBSPuppetHandle<Scalar,dimension> Super;
	USING_OPTIMTOOLS_MESH_TYPES(typename,Scalar,dimension);

public:
	inline BeakerPuppetHandle() { }

	inline BeakerPuppetHandle(Int id) :
		LBSPuppetHandle<Scalar,dimension>(id),
		m_constraint_weight(std::numeric_limits<Scalar>::max())
	{ }

	inline BeakerPuppetHandle(Int id, const Point& base_point, PuppetHandleType handle_type=PuppetHandleType::PIN) :
		LBSPuppetHandle<Scalar,dimension>(id, base_point, handle_type),
		m_constraint_weight(std::numeric_limits<Scalar>::max())
	{ }

	inline virtual ~BeakerPuppetHandle() { }

	using PuppetHandle::id;

	inline const Scalar& constraint_weight() const {
		return m_constraint_weight;
	}

	inline void set_constraint_weight(const Scalar& weight) {
		m_constraint_weight = weight;
	}

	inline virtual void serialize(serialization::Archive& ar) override {
		serialize_impl(ar);
	}

	template<typename Archive>
	inline void serialize(Archive& ar) {
		serialize_impl(ar);
	}

protected:
	template<typename Archive>
	Int serialize_fields(Archive& ar, Int id_counter) {
		enum {
			CONSTRAINT_WEIGHT = 0,
			NUM_FIELDS        = 1
		};

		ar("constraint_weight",id_counter+CONSTRAINT_WEIGHT) & m_constraint_weight;

		return id_counter + NUM_FIELDS;
	}

	template<typename Archive>
	void serialize_impl(Archive& ar) {
		ar.object([&](auto& ar) {
			Int id_counter = Super::serialize_fields(ar,0);
			serialize_fields(ar, id_counter);
		});
	}

private:
	Scalar m_constraint_weight;
};


/**
 * Specifies some subtypes used to customize an `AlternatingFASTARAPPuppet`
 * for Character Animator.
 */
template<typename Scalar>
struct BeakerPuppetTraits {
	typedef BeakerPuppetHandle<Scalar,2> Handle;
	typedef PuppetFigure<Scalar,Handle>  Figure;
	typedef BeakerMeshWarper<Scalar,2>   Warper;
};


/**
 * Creates a puppet backed by the Fast Automatic Skinning Transformations
 * algorithm, and intended for use in Character Animator.
 */
template<typename Scalar>
class BeakerPuppet : public AlternatingFASTARAPPuppet<Scalar, BeakerPuppetTraits<Scalar> > {
	typedef AlternatingFASTARAPPuppet<Scalar, BeakerPuppetTraits<Scalar> > Super;

	USING_OPTIMTOOLS_MESH_TYPES(typename,Scalar,2);
	typedef Eigen::Matrix<Scalar,Eigen::Dynamic,1>                         Vector;
	typedef typename Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Matrix;
	typedef Eigen::SparseMatrix<Scalar>                                    SparseMatrix;

public:
	typedef typename Super::Handle Handle;
	typedef typename Super::Figure Figure;
	typedef typename Super::Warper Warper;
	typedef typename Super::MeshingQuality MeshingQuality;

public:
	BeakerPuppet() :
		m_is_warper_handle_constraint_weights_dirty(false)
	{ }

	BeakerPuppet(const Figure& figure) :
		Super(figure),
		m_is_warper_handle_constraint_weights_dirty(true)
	{ }

	virtual ~BeakerPuppet() { }

	const Scalar& handle_constraint_weight(Int handle_id) const {
		return this->figure().handle(handle_id).constraint_weight();
	}

	void set_handle_constraint_weight(Int handle_id, const Scalar& handle_weight) {
		this->figure_ptr()->handle_ptr(handle_id)->set_constraint_weight(handle_weight);
		m_is_warper_handle_constraint_weights_dirty = true;
		this->dirty_deformed_mesh();
	}

	void solve_transforms() {
		this->warper()->solve_transforms();
	}
	
	void solve_rotations() {
		this->warper()->solve_rotations();
	}

	const Scalar& handle_strength_rotation_offset() const {
		return m_handle_strength_rotation_offset;
	}

	void set_handle_strength_rotation_offset(const Scalar& rotation_offset) {
		m_handle_strength_rotation_offset = rotation_offset;
		this->clear_warper_impl();
	}

	inline virtual void serialize(serialization::Archive& ar) override {
		serialize_impl(ar);
	}

	template<typename Archive>
	inline void serialize(Archive& ar) {
		serialize_impl(ar);
	}

protected:
	virtual shared_ptr<Warper> create_warper_from_args(
		const PointVector& points,
		const Matrix& skinning_weights,
		const PointVector& handle_points,
		const Eigen::SparseMatrix<Scalar>& A,
		const Eigen::VectorXd& w,
		const IntVector& group_inds,
		const std::vector<typename Warper::HandleType>& handle_types
	) const override {
		typename Warper::WarpParams warp_params;
		warp_params.handle_strength_rotation_offset() = m_handle_strength_rotation_offset;
		return make_shared<Warper>(
			points,
			skinning_weights,
			handle_points,
			A,
			w,
			group_inds,
			handle_types,
			warp_params
		);
	}

	void init_warper_handle_constraint_weights() {
		if(m_is_warper_handle_constraint_weights_dirty) {
			Vector handle_weights = convert_warper_handle_constraint_weights(this->figure(), this->handle_map());

			// This method is smart enough to check if the handle weights are
			// actually changed by this call, so we don't have to do it here.
			this->warper()->set_handle_constraint_weights(handle_weights);
			m_is_warper_handle_constraint_weights_dirty = false;
		}
	}

	virtual void init_warper_handle_attributes() override {
		Super::init_warper_handle_attributes();
		init_warper_handle_constraint_weights();
	}

	static Vector convert_warper_handle_constraint_weights(
		const Figure& figure,
		const PuppetMeshHandleMap<Scalar>& handle_map
	) {
		Vector handle_weights(figure.num_handles());
		figure.loop_handles([&](const Handle& handle) {
			Int hind = handle_map.handle_index(handle.id());
			handle_weights[hind] = handle.constraint_weight();
		});
		return handle_weights;
	}

	template<typename Archive>
	void serialize_impl(Archive& ar) {
		enum {
			LBS_PUPPET_BASE                           = 0,
			IS_WARPER_HANDLE_CONSTRAINT_WEIGHTS_DIRTY = 1,
			HANDLE_STRENGTH_ROTATION_OFFSET           = 2
		};
		ar.object([&](auto& ar) {
			auto lbs_puppet_base_ar = ar("lbs_puppet_base",LBS_PUPPET_BASE);
			Super::serialize_impl(lbs_puppet_base_ar);

			// this check for `has_warper` works for an input archive because
			// the superclass serialization will already have deserialized the
			// warper if it was in the archive.
			bool has_warper = this->is_warper_init();
			if(has_warper) {
				ar("is_warper_handle_constraint_weights_dirty",IS_WARPER_HANDLE_CONSTRAINT_WEIGHTS_DIRTY) & m_is_warper_handle_constraint_weights_dirty;
				ar("handle_strength_rotation_offset",HANDLE_STRENGTH_ROTATION_OFFSET) & m_handle_strength_rotation_offset;
			}
		});
	}

private:
	bool m_is_warper_handle_constraint_weights_dirty;
	Scalar m_handle_strength_rotation_offset{0.0};
};

END_OPTIMTOOLS_NAMESPACE
