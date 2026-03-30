#pragma once

/** \file
 * Defined a puppet with minimizes an ARAP energy over a subspace constructed
 * by linear blend skinning, as in Fast Automatic Skinning Transformations.
 */

#include "optimtools/all.h"

#include "puppet_warp/puppet_base.h"

BEGIN_OPTIMTOOLS_NAMESPACE

/**
 * Specifies the type of constraint that a handles places on the deformation
 * of a puppet.
 */
enum class PuppetHandleType {
	/**
	 * No explicit constraint, but the handle can impact the degrees for freedom
	 * for the deformation.  In practice, this is most useful with algorithms
	 * using linear blend skinning, where a handle of this type can add stiffness
	 * to a region of the mesh, or ensure that a set of vertices always transform
	 * affinely.
	 */
	FREE  = 0,

	/**
	 * The handle specifies the location (but not the rotation or scale) of a
	 * point on the mesh.
	 */
	PIN   = 1,

	/**
	 * The handle specifies the rotation and scale (but not the translation) of
	 * a region of the mesh.
	 */
	SLIDING = 2,

	/**
	 * The handle explicitly specifies the location, rotation, and scale of a
	 * region of the mesh.
	 */
	FIXED = 3
};


template<typename Scalar, Int dimension>
class LBSPuppetHandle : public PuppetHandle {
	USING_OPTIMTOOLS_MESH_TYPES(typename,Scalar,dimension);

public:
	inline LBSPuppetHandle() { }

	inline LBSPuppetHandle(Int id) :
		PuppetHandle(id),
		m_base_point(Point::Zero()),
		m_handle_type(PuppetHandleType::PIN)
	{ }

	inline LBSPuppetHandle(Int id, const Point& base_point, PuppetHandleType handle_type=PuppetHandleType::PIN) :
		PuppetHandle(id),
		m_base_point(base_point),
		m_handle_type(handle_type)
	{ }

	inline virtual ~LBSPuppetHandle() { }

	using PuppetHandle::id;

	inline const Point& base_point() const {
		return m_base_point;
	}

	inline void set_base_point(const Point& base_point) {
		m_base_point = base_point;
	}

	inline PuppetHandleType handle_type() const {
		return m_handle_type;
	}

	inline void set_handle_type(PuppetHandleType handle_type) {
		m_handle_type = handle_type;
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
			ID                = 0,
			BASE_POINT        = 1,
			HANDLE_TYPE       = 2,
			NUM_FIELDS        = 3
		};

		ar("id",id_counter+ID) & id();
		ar("base_point",id_counter+BASE_POINT) & m_base_point;
		ar("handle_type",id_counter+HANDLE_TYPE) & m_handle_type;

		return id_counter + NUM_FIELDS;
	}

	template<typename Archive>
	void serialize_impl(Archive& ar) {
		ar.object([&](auto& ar) {
			serialize_fields(ar,0);
		});
	}

private:
	Point m_base_point;
	PuppetHandleType m_handle_type;
};

//----------------------------------------------------------------------------//

/**
 * Defaults some types to be used in the defaule implementation as an
 * `AlternatingFASTARAPPuppet`.  You can provide a class with different
 * typedefs if you need to add additional functionality (see `BeakerPuppet`
 * and `BeakerPuppetTraits` for an example of this).
 */
template<typename Scalar>
struct DefaultFASTARAPPuppetTraits {
	typedef LBSPuppetHandle<Scalar,2>               Handle;
	typedef PuppetFigure<Scalar,Handle>             Figure;
	typedef AlternatingFASTARAPMeshWarper<Scalar,2> Warper;
};


/**
 * Creates a puppet backed by the Fast Automatic Skinning Transformations
 * algorithm.
 */
template<typename Scalar, class Traits=DefaultFASTARAPPuppetTraits<Scalar> >
class AlternatingFASTARAPPuppet : public Puppet<Scalar>, public serialization::Serializable {
	USING_OPTIMTOOLS_MESH_TYPES(typename,Scalar,2);
	typedef Eigen::Matrix<Scalar,Eigen::Dynamic,1>                         Vector;
	typedef typename Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Matrix;
	typedef Eigen::SparseMatrix<Scalar>                                    SparseMatrix;

	typedef typename Puppet<Scalar>::HandleDragger HandleDragger;
	typedef typename Traits::Warper::HandleType    WarperHandleType;
	typedef PuppetMarker<Scalar>                   Marker;

public:
	typedef typename Traits::Handle                              Handle;
	typedef typename Traits::Figure                              Figure;
	typedef typename Traits::Warper                              Warper;
	typedef typename AdaptivePuppetFigureMesher<Scalar>::Quality MeshingQuality;

public:
	AlternatingFASTARAPPuppet() { }

	AlternatingFASTARAPPuppet(const AlternatingFASTARAPPuppet& other) :
		Puppet<Scalar>(other),
		m_figure(make_shared<Figure>(*other.m_figure)),
		m_meshing_quality(other.m_meshing_quality),
		m_skinning_weights(other.m_skinning_weights),
		m_is_warper_handle_types_dirty(other.m_is_warper_handle_types_dirty)
	{
		if(other.m_warper) {
			m_warper = make_shared<Warper>(*(other.m_warper));
		}
	}

	AlternatingFASTARAPPuppet(const Figure& figure) {
		shared_ptr<Figure> figure_copy = make_shared<Figure>(figure);
		Puppet<Scalar>::init(figure_copy);
		m_figure = figure_copy; 
	}

	virtual ~AlternatingFASTARAPPuppet() { }

	const Figure& figure() const {
		return *m_figure;
	}

	Scalar mesh_energy() const {
		Assert(is_warper_init()) << "cannot call `mesh_energy()` before the warper is initialized" << raise;
		return m_warper->mesh_energy();
	}

	virtual Int add_point_handle(const Point& p, const Scalar& threshold_dist) override {
		Int handle_id = Puppet<Scalar>::add_point_handle(p, threshold_dist);
		if(handle_id >= 0) {
			set_base_handle_point(handle_id, p);
		}
		return handle_id;
	}

	virtual Int add_segment_handle(
		const Point& p1,
		const Point& p2,
		const Scalar& threshold_dist,
		const Scalar& pslg_threshold_dist
	) override {
		Int handle_id = Puppet<Scalar>::add_segment_handle(p1, p2, threshold_dist, pslg_threshold_dist);
		if(handle_id >= 0) {
			set_base_handle_point(handle_id, (p1+p2)/Scalar(2));
		}
		return handle_id;
	}

	PuppetHandleType handle_type(Int handle_id) const {
		return figure().handle(handle_id).handle_type();
	}

	void set_handle_type(Int handle_id, PuppetHandleType handle_type) {
		figure_ptr()->handle_ptr(handle_id)->set_handle_type(handle_type);
		if(is_warper_init()) {
			m_is_warper_handle_types_dirty = true;
		}
		this->dirty_deformed_mesh();
	}

	const Point& base_handle_point(Int handle_id) const {
		assert(is_warper_init());
		Int handle_ind = this->handle_map().handle_index(handle_id);
		return m_warper->base_handle_point(handle_ind);
	}

	void set_base_handle_point(Int handle_id, const Point& base_point) {
		figure_ptr()->handle_ptr(handle_id)->set_base_point(base_point);
		clear_warper_impl();
	}

	const MeshingQuality& meshing_quality() const {
		return m_meshing_quality;
	}

	MeshingQuality& meshing_quality() {
		return m_meshing_quality;
	}

	const Matrix& skinning_weights() const {
		assert(is_warper_init());
		return m_skinning_weights;
	}

	virtual void move_dragger(const HandleDragger& dragger, const Point& p) override {
		assert(is_warper_init());
		assert(this->is_valid_dragger(dragger));
		Int handle_ind = this->handle_map().handle_index(dragger.handle_id());
		m_warper->set_handle_point(handle_ind, p);
		this->dirty_deformed_mesh();
	}

	Point handle_point(Int handle_id) const {
		Assert(is_warper_init()) << "cannot call `handle_point()` before the warper is initialized" << raise;
		Int handle_ind = this->handle_map().handle_index(handle_id);
		return m_warper->handle_point(handle_ind);
	}

	void set_handle_point(Int handle_id, const Point& p) {
		Assert(is_warper_init()) << "cannot call `set_handle_point()` before the warper is initialized" << raise;
		init_warper_handle_attributes();
		Int handle_ind = this->handle_map().handle_index(handle_id);
		m_warper->set_handle_point(handle_ind, p);
		this->dirty_deformed_mesh();
	}

	Affine target_handle_affine(Int handle_id) const {
		Assert(is_warper_init()) << "cannot call `target_handle_affine()` before the warper is initialized" << raise;
		Int handle_ind = this->handle_map().handle_index(handle_id);
		return m_warper->target_handle_affine(handle_ind);
	}
	
	Affine deformed_handle_affine(Int handle_id) const {
		Assert(is_warper_init()) << "cannot call `deformed_handle_affine()` before the warper is initialized" << raise;
		Int handle_ind = this->handle_map().handle_index(handle_id);
		return m_warper->deformed_handle_affine(handle_ind);
	}

	void set_handle_affine(Int handle_id, const Affine& affine) {
		Assert(is_warper_init()) << "cannot call `set_handle_affine()` before the warper is initialized" << raise;
		init_warper_handle_attributes();
		Int handle_ind = this->handle_map().handle_index(handle_id);
		m_warper->set_handle_affine(handle_ind, affine);
		this->dirty_deformed_mesh();
	}

	void set_target_handle_affine(Int handle_id, const Affine& tr) {
		Assert(is_warper_init()) << "cannot call `set_target_handle_affine()` before the warper is initialized" << raise;
		init_warper_handle_attributes();
		Int handle_ind = this->handle_map().handle_index(handle_id);
		m_warper->set_target_handle_affine(handle_ind, tr);
		this->dirty_deformed_mesh();
	}
	
	void set_deformed_handle_affine(Int handle_id, const Affine& tr) {
		Assert(is_warper_init()) << "cannot call `set_deformed_handle_affine()` before the warper is initialized" << raise;
		init_warper_handle_attributes();
		Int handle_ind = this->handle_map().handle_index(handle_id);
		m_warper->set_deformed_handle_affine(handle_ind, tr);
		this->dirty_deformed_mesh();
	}

	template<typename PointVectorDerived, typename AffineVectorDerived>
	void get_mesh_points_from_handle_affines(
		const Eigen::DenseBase<PointVectorDerived>& out_mesh_points_const,
		const Eigen::DenseBase<AffineVectorDerived>& handle_affines
	) const {
		Assert(is_warper_init()) << "cannot call `get_mesh_points_from_handle_affines()` before the warper is initialized" << raise;
		m_warper->get_mesh_points_from_handle_affines(out_mesh_points_const, handle_affines);
	}

	template<typename AffineVectorDerived>
	Affine transformation_at_marker_from_handle_affines(
		const Marker& marker,
		const Eigen::DenseBase<AffineVectorDerived>& handle_affines
	) const {
		Assert(is_warper_init()) << "cannot call `transformation_at_marker_from_handle_affines()` before the warper is initialized" << raise;
		Assert(marker.has_triangle()) << "cannot call `transformation_at_marker()` on marker without triangle+barycentric information" << raise;
		const Triangle& tri = this->base_mesh().triangles()[marker.triangle_ind()];

		if(this->num_handles() == 0) {
			Affine tr = Affine::Identity();
			tr.translation() = marker.base_point();
			return tr;
		} else {
			const Point& p1 = this->base_mesh().points()[tri[0]];
			const Point& p2 = this->base_mesh().points()[tri[1]];
			const Point& p3 = this->base_mesh().points()[tri[2]];

			// determine where the associated triangle in the defomed mesh goes
			Point q1 = Point::Zero();
			Point q2 = Point::Zero();
			Point q3 = Point::Zero();
			for(Int handle_ind = 0; handle_ind < this->num_handles(); ++handle_ind) {
				// get the handle affine transform
				Affine handle_tr = handle_affines[handle_ind];
				Point handle_base_point = base_handle_point(handle_ind);

				// get the handle weights at each of the triangle's vertices
				Scalar w1 = skinning_weights()(tri[0], handle_ind);
				Scalar w2 = skinning_weights()(tri[1], handle_ind);
				Scalar w3 = skinning_weights()(tri[2], handle_ind);

				// accumulate to the deformed position of each vertex
				q1 += w1 * (handle_tr * (p1 - handle_base_point).matrix()).array();
				q2 += w2 * (handle_tr * (p2 - handle_base_point).matrix()).array();
				q3 += w3 * (handle_tr * (p3 - handle_base_point).matrix()).array();
			}
			Point q = marker.barycentric_coord_1()*q1
			        + marker.barycentric_coord_2()*q2
			        + marker.barycentric_coord_3()*q3;
		
			Affine result = affine_triangle_mapping(p1, p2, p3, q1, q2, q3);
			result.translation() = q;
			return result;
		}
	}

	void reset_pose() {
		Assert(is_warper_init()) << "cannot call `reset_pose()` before the warper is initialized" << raise;
		m_warper->reset_pose();
		this->dirty_deformed_mesh();
	}

	const shared_ptr<Warper>& warper() const {
		return m_warper;
	}

	virtual void init_warper() override {
		Puppet<Scalar>::init_warper();
		if(this->num_handles() > 0) {
			init_warper_handle_attributes();
		}
	}

	inline virtual void serialize(serialization::Archive& ar) override {
		serialize_impl(ar);
	}

	template<typename Archive>
	inline void serialize(Archive& ar) {
		serialize_impl(ar);
	}

public:
	template<typename Callback>
	static shared_ptr<Warper> create_warper_with_callback(
		shared_ptr<PuppetMesh<Scalar> >& out_base_mesh,
		shared_ptr<PuppetMeshHandleMap<Scalar> >& out_handle_map,
		Matrix& out_skinning_weights,
		const shared_ptr<Figure>& figure,
		const MeshingQuality& meshing_quality,
		const Callback& callback
	) {
		return create_warper_with_callback(
			out_base_mesh,
			out_handle_map,
			out_skinning_weights,
			figure,
			meshing_quality,
			callback,
			[](
				const PointVector& points,
				const Matrix& skinning_weights,
				const PointVector& handle_points,
				const Eigen::SparseMatrix<Scalar>& A,
				const Eigen::VectorXd& w,
				const IntVector& group_inds,
				const std::vector<WarperHandleType>& handle_types
			) {
				return make_shared<Warper>(
					points,
					skinning_weights,
					handle_points,
					A,
					w,
					group_inds,
					handle_types
				);
			}
		);
	}

	template<typename Callback, typename CreateWarperFunc>
	static shared_ptr<Warper> create_warper_with_callback(
		shared_ptr<PuppetMesh<Scalar> >& out_base_mesh,
		shared_ptr<PuppetMeshHandleMap<Scalar> >& out_handle_map,
		Matrix& out_skinning_weights,
		const shared_ptr<Figure>& figure,
		const MeshingQuality& meshing_quality,
		const Callback& callback,
		const CreateWarperFunc& create_warper_func
	) {
		Assert(figure->num_handles() >= 1) << "cannot initialize a FAST ARAP warper with zero handles" << raise;

		// Solve for the base mesh.  Doing so involves solving for the cotangent
		// Laplacian and the bounded biharmonic skinning weights, so remember them
		// so they can be reused later in this method.
		Eigen::SparseMatrix<Scalar> A;
		Eigen::VectorXd w;
		AdaptivePuppetFigureMesher<Scalar> mesher(figure, [&](
			Matrix& inout_weights,
			const PuppetFigureShape<Scalar>& figure,
			const PuppetMesh<Scalar>& mesh,
			const PuppetMeshHandleMap<Scalar>& handle_map,
			const AdaptivePuppetFigureMesher<Scalar>& mesher
		) {
			callback(mesher);

			const PointVector& points = mesh.points();
			const TriangleVector& triangles = mesh.triangles();
			const std::vector<IntVector>& handle_vertices = handle_map.handle_vertices();

			cotangent_laplacian<Scalar,2>::mesh_laplacian(A, w, points, triangles);
			SparseMatrix L = A.transpose() * w.asDiagonal() * A;

			Vector mass;
			cotangent_laplacian<Scalar,2>::lumped_mass(mass, points, triangles);
			auto inv_mass = mass.cwiseInverse().asDiagonal();

			BoundedBiharmonicSkinningSolver<Scalar> solver;
			solver.init_from_invmass_and_stiffness(inv_mass, L, handle_vertices);

			if(inout_weights.size() > 0) {
				// set an initial guess to speed up the solver
				solver.set_initial_guess(inout_weights);
			}
			//const_cast<AdaptivePuppetFigureMesher<Scalar>&>(mesher).skinning_weights() = solver.skinning_weights();
			callback(mesher);
			while(solver.iterate()) {
				//const_cast<AdaptivePuppetFigureMesher<Scalar>&>(mesher).skinning_weights() = solver.skinning_weights();
				callback(mesher);
			}
			inout_weights = solver.skinning_weights();
		});
		mesher.set_quality(meshing_quality);
		mesher.solve();
		out_base_mesh = mesher.mesh();
		out_handle_map = mesher.handle_map();
		out_skinning_weights = mesher.skinning_weights();

		const PointVector& points = out_base_mesh->points();
		const TriangleVector& triangles = out_base_mesh->triangles();

		// determine the rotation groups to be used in the FAST algorithm.
		// Ideally, we want two clusters per handle, but not more than one cluster per three points
		Int num_clusters = (out_skinning_weights.cols() <= 1) ? 1 : std::min(points.size()/3, 2*out_skinning_weights.cols());
		IntVector group_inds; // A and w are also modified to match the order implied by group_inds
		if(num_clusters <= 1) {
			// if there's only one cluster, then group_inds is trivial
			group_inds.resize(2);
			group_inds[0] = 0;
			group_inds[1] = A.rows();
		} else {
			typename fast_arap_utils<Scalar>::KMeansClustersSolver cluster_solver(out_skinning_weights.transpose(), num_clusters);
			cluster_solver.set_min_cluster_size(2);
			IntVector point_clusters = cluster_solver.solve();
			num_clusters = cluster_solver.num_clusters(); // small clusters may have been removed

			Eigen::SparseMatrix<Scalar> A_copy = A;
			Vector w_copy = w;
			fast_arap_utils<Scalar>::apply_clusters_to_laplacian(A, w, group_inds, point_clusters, A_copy, w_copy, triangles);
			assert(group_inds.size() == num_clusters+1);
		}

		// Extract the `WarperHandleType`s from the `PuppetHandle`s
		std::vector<WarperHandleType> handle_types = convert_warper_handle_types(*figure, *out_handle_map);

		// Extract the handle base pointe from the `PuppetHandle`s
		PointVector handle_base_points = convert_warper_handle_base_points(*figure, *out_handle_map);

		// Create the warper.
		return create_warper_func(
			points,
			out_skinning_weights,
			handle_base_points,
			A,
			w,
			group_inds,
			handle_types
		);
	}

	static shared_ptr<Warper> create_warper(
		shared_ptr<PuppetMesh<Scalar> >& out_base_mesh,
		shared_ptr<PuppetMeshHandleMap<Scalar> >& out_handle_map,
		Matrix& out_skinning_weights,
		const shared_ptr<Figure>& figure,
		const MeshingQuality& meshing_quality
	) {
		return create_warper_with_callback(
			out_base_mesh,
			out_handle_map,
			out_skinning_weights,
			figure,
			meshing_quality,
			[](const auto& optimizer) { } // no-op callback
		);
	}

	template<typename CreateWarperFunc>
	static shared_ptr<Warper> create_warper(
		shared_ptr<PuppetMesh<Scalar> >& out_base_mesh,
		shared_ptr<PuppetMeshHandleMap<Scalar> >& out_handle_map,
		Matrix& out_skinning_weights,
		const shared_ptr<Figure>& figure,
		const MeshingQuality& meshing_quality,
		const CreateWarperFunc& create_warper_func
	) {
		return create_warper_with_callback(
			out_base_mesh,
			out_handle_map,
			out_skinning_weights,
			figure,
			meshing_quality,
			[](const auto& optimizer) { }, // no-op callback
			create_warper_func
		);
	}

	static std::vector<WarperHandleType> convert_warper_handle_types(
		const Figure& figure,
		const PuppetMeshHandleMap<Scalar>& handle_map
	) {
		std::vector<WarperHandleType> handle_types(figure.num_handles());
		figure.loop_handles([&](const Handle& handle) {
			Int hind = handle_map.handle_index(handle.id());
			switch(handle.handle_type()) {
			case PuppetHandleType::FREE:
				handle_types.at(hind) = WarperHandleType::FREE;
				break;
			case PuppetHandleType::PIN:
				handle_types.at(hind) = WarperHandleType::PIN;
				break;
			case PuppetHandleType::SLIDING:
				handle_types.at(hind) = WarperHandleType::SLIDING;
				break;
			case PuppetHandleType::FIXED:
				handle_types.at(hind) = WarperHandleType::FIXED;
				break;
			default:
				Error() << "AlternatingFASTARAPPuppet does not support PuppetHandleType " << (Int)handle.handle_type() << raise;
			}
		});
		return handle_types;
	}

	static PointVector convert_warper_handle_base_points(
		const Figure& figure,
		const PuppetMeshHandleMap<Scalar>& handle_map
	) {
		PointVector base_points(figure.num_handles());
		figure.loop_handles([&](const Handle& handle) {
			Int hind = handle_map.handle_index(handle.id());
			base_points[hind] = handle.base_point();
		});
		return base_points;
	}

protected:
	const shared_ptr<Figure>& figure_ptr() const {
		return m_figure;
	}

	virtual void get_mesh_points(PointVector& out_points) const override {
		m_warper->get_mesh_points(out_points);
	}

	virtual bool iterate_warper_impl() override {
		return m_warper->iterate();
	}

	virtual bool is_warper_init() const override {
		return static_cast<bool>(m_warper);
	}

	virtual shared_ptr<Warper> create_warper_from_args(
		const PointVector& points,
		const Matrix& skinning_weights,
		const PointVector& handle_points,
		const Eigen::SparseMatrix<Scalar>& A,
		const Eigen::VectorXd& w,
		const IntVector& group_inds,
		const std::vector<WarperHandleType>& handle_types
	) const {
		return make_shared<Warper>(
			points,
			skinning_weights,
			handle_points,
			A,
			w,
			group_inds,
			handle_types
		);
	}

	virtual void init_warper_impl(
		shared_ptr<PuppetMesh<Scalar> >& out_base_mesh,
		shared_ptr<PuppetMeshHandleMap<Scalar> >& out_handle_map
	) override {
		m_warper = create_warper(
			out_base_mesh,
			out_handle_map,
			m_skinning_weights,
			figure_ptr(),
			meshing_quality(),
			[this](const auto&... params) { return this->create_warper_from_args(params...); }
		);
		m_is_warper_handle_types_dirty = false;
	}

	virtual void clear_warper_impl() override {
		m_warper.reset();
	}

	void init_warper_handle_types() {
		if(m_is_warper_handle_types_dirty) {
			std::vector<WarperHandleType> handle_types = convert_warper_handle_types(this->figure(), this->handle_map());

			// This method is smart enough to check if the handle types are
			// actually changed by this call, so we don't have to do it here.
			m_warper->set_handle_types(handle_types);
			m_is_warper_handle_types_dirty = false;
		}
	}

	virtual void init_warper_handle_attributes() {
		init_warper_handle_types();
	}

	template<typename Archive>
	void serialize_impl(Archive& ar) {
		enum {
			PUPPET_BASE                               = 0,
			FIGURE                                    = 1,
			MESHING_QUALITY                           = 2,
			SKINNING_WEIGHTS                          = 3,
			FAST_ARAP_WARPER                          = 4,
			IS_WARPER_HANDLE_TYPES_DIRTY              = 5
		};
		ar.object([&](auto& ar) {
			auto puppet_base_ar = ar("puppet_base",PUPPET_BASE);
			Puppet<Scalar>::serialize_impl(puppet_base_ar);
			
			ar("figure",FIGURE) & m_figure;
			if(ar.is_input()) {
				this->set_figure_shape_ptr(m_figure);
			}
			ar("meshing_quality",MESHING_QUALITY) & m_meshing_quality;

			bool has_warper = ar.is_input() ? ar.has_field("fast_arap_warper",FAST_ARAP_WARPER) : is_warper_init();
			if(has_warper) {
				ar("skinning_weights",SKINNING_WEIGHTS) & m_skinning_weights;
				ar("fast_arap_warper",FAST_ARAP_WARPER) & m_warper;
				ar("is_warper_handle_types_dirty",IS_WARPER_HANDLE_TYPES_DIRTY) & m_is_warper_handle_types_dirty;
			}
		});
	}

private:
	shared_ptr<Figure> m_figure; // points to same figure as in superclass (but has a more specific type)

	MeshingQuality m_meshing_quality;
	Matrix m_skinning_weights;
	shared_ptr<Warper> m_warper;
	bool m_is_warper_handle_types_dirty;
};

END_OPTIMTOOLS_NAMESPACE

