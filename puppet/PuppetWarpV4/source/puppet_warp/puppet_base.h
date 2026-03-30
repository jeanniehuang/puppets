#pragma once

/** \file
 * Provides a base classe representing deformable puppets.  Concrete puppets
 * implementations are used by subclasses of this class.
 *
 * Most of the required PuppetWarp functionality is handled elsewhere by
 * `PuppetFigure`, `PuppetMesh`, optimtools mesh deformation functionality,
 * and various utility functions.  Subclasses of `Puppet` will normally tie
 * this functionality together, and handle the bookkeeping needed to manage
 * the puppet's state as the figure is altered or the mesh is deformed.
 */

#include "optimtools/all.h"

#include "puppet_warp/puppet_accelerator.h"
#include "puppet_warp/puppet_handle.h"
#include "puppet_warp/puppet_figure.h"
#include "puppet_warp/puppet_mesh.h"
#include "puppet_warp/puppet_utils.h"

BEGIN_OPTIMTOOLS_NAMESPACE

/**
 * A `PuppetMarker` provides a lightweight way of attaching a point to a
 * puppet.  Unlike a handle a marker's position can only be passively
 * querried and can not be used modify the state of the puppet.  The
 * benefit of using a marker is that they are significantly more efficient
 * than handles, and thus should be preferred wherever applicable.
 */
template<typename Scalar>
class PuppetMarker : public serialization::Serializable {
	USING_OPTIMTOOLS_MESH_TYPES(typename,Scalar,2);
	typedef Eigen::Array<Scalar,3,1> BarycentricPoint;

public:
	PuppetMarker() { }

	PuppetMarker(const Point& base_point) :
		m_base_point(base_point),
		m_triangle_ind(-1)
	{ }

	PuppetMarker(
		const Point& base_point,
		Int triangle_ind,
		const Scalar& barycentric_coord_1,
		const Scalar& barycentric_coord_2,
		const Scalar& barycentric_coord_3
	) :
		m_base_point(base_point)
	{
		set_triangle(triangle_ind, barycentric_coord_1, barycentric_coord_2, barycentric_coord_3);
	}

	const Point& base_point() const {
		return m_base_point;
	}

	bool has_triangle() const {
		return m_triangle_ind >= 0;
	}

	void set_triangle(
		Int triangle_ind,
		const Scalar& barycentric_coord_1,
		const Scalar& barycentric_coord_2,
		const Scalar& barycentric_coord_3
	) {
		m_triangle_ind = triangle_ind;
		m_barycentric[0] = barycentric_coord_1;
		m_barycentric[1] = barycentric_coord_2;
		m_barycentric[2] = barycentric_coord_3;
	}

	void unset_triangle() {
		m_triangle_ind = -1;
	}

	Int triangle_ind() const {
		return m_triangle_ind;
	}

	Int& triangle_ind() {
		return m_triangle_ind;
	}

	const Scalar& barycentric_coord_1() const {
		return m_barycentric[0];
	}

	Scalar& barycentric_coord_1() {
		return m_barycentric[0];
	}

	const Scalar& barycentric_coord_2() const {
		return m_barycentric[1];
	}

	Scalar& barycentric_coord_2() {
		return m_barycentric[1];
	}

	const Scalar& barycentric_coord_3() const {
		return m_barycentric[2];
	}

	Scalar& barycentric_coord_3() {
		return m_barycentric[2];
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
	void serialize_impl(Archive& ar) {
		enum {
			BASE_POINT   = 0,
			TRIANGLE_IND = 1,
			BARYCENTRIC  = 2
		};

		ar.object([&](auto& ar) {
			ar("base_point",BASE_POINT) & m_base_point;
			ar("triangle_ind",TRIANGLE_IND) & m_triangle_ind;
			ar("barycentric",BARYCENTRIC) & m_barycentric;
		});
	}

private:
	Point m_base_point; // position in the figure / base mesh
	Int m_triangle_ind;
	BarycentricPoint m_barycentric;
};


/**
 * An abstract superclass for puppets.  This primarily handles the
 * functionality for modifying and managing the puppet's state, with
 * the actual meshing and deformation handled by the subclass.
 */
template<typename Scalar>
class Puppet {
	USING_OPTIMTOOLS_MESH_TYPES(typename,Scalar,2);
	typedef PuppetMarker<Scalar> Marker;
	typedef typename std::map<Int, Marker> MarkerMap;

public:
	/**
	 * A utility class to store the information required to drag a handle
	 * around.
	 */
	class HandleDragger {
	public:
		HandleDragger() {
			reset();
		}

		void reset() {
			m_handle_id = -1;
		}

	//private: // these would be private, but they're needed by subclasses
		HandleDragger(Int handle_id, Point initial_point) :
			m_handle_id(handle_id),
			m_initial_point(initial_point)
		{ }

		const Int& handle_id() const {
			return m_handle_id;
		}

		const Point& initial_point() const {
			return m_initial_point;
		}

	private:
		Int m_handle_id;
		Point m_initial_point;
	};

protected:
	Puppet() { }

	Puppet(const Puppet& other) :
		m_figure(other.m_figure->clone()),
		m_base_mesh(other.m_base_mesh), // `m_base_mesh` is overwritten, but never modified
		m_base_mesh_geometry(other.m_base_mesh_geometry), // see comment for `m_base_mesh`
		m_is_deformed_mesh_dirty(other.m_is_deformed_mesh_dirty),
		m_marker_counter(other.m_marker_counter),
		m_markers(other.m_markers)
	{
		if(other.m_deformed_mesh) {
			// The deformed mesh shares its topology with the deformed mesh, so
			// we don't have to clone everything.
			assert(m_base_mesh);
			shared_ptr<PointVector> deformed_points = make_shared<PointVector>(other.m_deformed_mesh->points());
			m_deformed_mesh = make_shared<PuppetMesh<Scalar> >(
				deformed_points,
				base_mesh().triangles_ptr(),
				base_mesh().point_handles_ptr()
			);
		}
		if(other.m_handle_map) {
			m_handle_map = make_shared<PuppetMeshHandleMap<Scalar> >(*(other.m_handle_map));
		}
	}

	Puppet(const shared_ptr<PuppetFigureShape<Scalar> >& figure) {
		init(figure);
	}

	void init(const shared_ptr<PuppetFigureShape<Scalar> >& figure) {
		m_figure = figure;
		m_base_mesh = make_shared<PuppetMesh<Scalar> >();
		m_is_deformed_mesh_dirty = true;
		m_marker_counter = 0;
	}
	
	virtual ~Puppet() { }

public:
	bool is_init() const {
		return (bool)m_figure;
	}

	const PuppetFigureShape<Scalar>& figure_shape() const {
		return *m_figure;
	}

	const PuppetMesh<Scalar>& base_mesh() const {
		return *m_base_mesh;
	}

	const PuppetMeshHandleMap<Scalar>& handle_map() const {
		return *m_handle_map;
	}

	const PuppetMesh<Scalar>& deformed_mesh() const {
		Assert(is_warper_init() || num_handles() == 0) << "cannot call `deformed_mesh()` before the warper is initialized" << raise;
		if(m_is_deformed_mesh_dirty) {
			if(num_handles() >= 1) {
				// There are handles, so defer to the subclass to get the
				// deformed mesh from its internal warper.
				get_mesh_points(const_cast<PointVector&>(m_deformed_mesh->points()));
			} else {
				// No handles, so it doesn't make sense to run the warper.
				// Just set the deformed mesh to the base mesh.
				const_cast<PointVector&>(m_deformed_mesh->points()) = m_base_mesh->points();
			}
			const_cast<bool&>(m_is_deformed_mesh_dirty) = false;
		}
		return *m_deformed_mesh;
	}

	Int num_handles() const {
		return m_figure->num_handles();
	}

	virtual Int add_point_handle(const Point& p, const Scalar& threshold_dist) {
		Int handle_id = m_figure->add_point_handle(p, threshold_dist);
		dirty_mesh();
		return handle_id;
	}
	
	virtual Int add_segment_handle(
		const Point& p1,
		const Point& p2,
		const Scalar& threshold_dist,
		const Scalar& pslg_threshold_dist
	) {
		Int handle_id = m_figure->add_segment_handle(p1, p2, threshold_dist, pslg_threshold_dist);
		dirty_mesh();
		return handle_id;
	}

	virtual Int add_segment_handle(const Point& p1, const Point& p2, const Scalar& threshold_dist) {
		return add_segment_handle(p1, p2, threshold_dist, threshold_dist);
	}

	void remove_handle(Int handle_id) {
		m_figure->remove_handle(handle_id);
		dirty_mesh();
	}

	HandleDragger drag_handle(Int handle_id, const Point& initial_p) {
		init_warper();
		return HandleDragger(handle_id, initial_p);
	}

	virtual bool is_valid_dragger(const HandleDragger& dragger) const {
		// the `>= 0` check is not strictly necessary, but gives a quick and
		// efficient check for the most common case before trying the full test.
		return dragger.handle_id() >= 0 && m_figure->has_handle(dragger.handle_id());
	}

	virtual void move_dragger(const HandleDragger& dragger, const Point& p) = 0;

	virtual void init_warper() {
		if(!is_warper_init()) {
			if(num_handles() >= 1) {
				// There are handles, so it's ok to call the subclass init warper
				init_warper_impl(m_base_mesh, m_handle_map);
			} else {
				// No handles, so it doesn't make sense to init the warper.
				// instead just create a minimal triangulation and use it for the
				// base mesh.
				BasicPuppetFigureMesher<Scalar> mesher(figure_shape_ptr());
				mesher.set_minimal_meshing();
				m_base_mesh = mesher.solve();

				// no handles, so initialize an empty handle map
				m_handle_map = make_shared<PuppetMeshHandleMap<Scalar> >();
			}

			m_base_mesh_geometry = make_shared<PuppetMeshGeometryBruteForce<Scalar> >(m_base_mesh);

			shared_ptr<PointVector> deformed_points = make_shared<PointVector>(base_mesh().points());
			m_deformed_mesh = make_shared<PuppetMesh<Scalar> >(
				deformed_points,
				base_mesh().triangles_ptr(),
				base_mesh().point_handles_ptr()
			);
			remesh_markers();
			m_is_deformed_mesh_dirty = true;
		}
	}

	bool iterate_warper() {
		init_warper();
		dirty_deformed_mesh();
		if(num_handles() >= 1) {
			bool ret = iterate_warper_impl();
			return ret;
		} else {
			// no handles, so just set the deformed mesh to the base mesh
			return false;
		}
	}

	/**
	 * Creates a new marker located at the point `p` on the base shape of
	 * this puppet (i.e. the shape of the figure), adds it to the list of
	 * markers managed by this puppet, and returns an ID that can be used
	 * later to reference the marker.
	 */
	Int add_marker_to_base_shape(const Point& p) {
		Marker marker(p);
		remesh_marker(marker);
		Int marker_id = m_marker_counter;
		m_markers[marker_id] = marker;
		++m_marker_counter;
		return marker_id;
	}

	/**
	 * Returns `true` if this puppet is currently managing a marker with the
	 * given ID, and `false` otherwise.
	 */
	bool has_marker(Int marker_id) const {
		return m_markers.find(marker_id) != m_markers.end();
	}

	/**
	 * Returns the marker managed by this puppet with the associated ID.
	 * throws an exception if `has_marker(marker_id)` is `false`.
	 */
	const Marker& get_marker(Int marker_id) const {
		return m_markers.at(marker_id);
	}

	/**
	 * Returns the position of the given marker on the deformed mesh.
	 * The marker does *not* need to be managed by this puppet.
	 */
	Point deformed_marker_position(const Marker& marker) const {
		if(num_handles() == 0) {
			// If there are no handles, then the transformation is always the
			// identity, so we don't need a mesh nor a warper.
			return marker.base_point();
		} else {
			Assert(is_warper_init()) << "cannot call `deformed_marker_position()` if the puppet has not been meshed and made ready to warp" << raise;
			Assert(marker.has_triangle()) << "cannot call `deformed_marker_position()` on marker without triangle+barycentric information" << raise;
			const Triangle& tri = base_mesh().triangles()[marker.triangle_ind()];
			const Point& q1 = deformed_mesh().points()[tri[0]];
			const Point& q2 = deformed_mesh().points()[tri[1]];
			const Point& q3 = deformed_mesh().points()[tri[2]];
			return from_triangle_barycentric(
				marker.barycentric_coord_1(),
				marker.barycentric_coord_2(),
				marker.barycentric_coord_3(),
				q1, q2, q3
			);
		}
	}

	/**
	 * Returns the position on the deformed mesh of the marker with the given ID.
	 */
	Point deformed_marker_position(Int marker_id) const {
		return deformed_marker_position(get_marker(marker_id));
	}

	/**
	 * Returns the affine transformation of the deformed mesh at the given
	 * marker.  The marker does *not* need to be managed by this puppet.
	 */
	Affine transformation_at_marker(const Marker& marker) const {
		if(num_handles() == 0) {
			// If there are no handles, then the transformation is always the
			// identity, so we don't need a mesh nor a warper.
			Affine tr = Affine::Identity();
			tr.translation() = marker.base_point();
			return tr;
		} else {
			Assert(is_warper_init()) << "cannot call `transformation_at_marker()` if the puppet has not been meshed and made ready to warp" << raise;
			Assert(marker.has_triangle()) << "cannot call `transformation_at_marker()` on marker without triangle+barycentric information" << raise;
			const Triangle& tri = base_mesh().triangles()[marker.triangle_ind()];
			const Point& p1 = base_mesh().points()[tri[0]];
			const Point& p2 = base_mesh().points()[tri[1]];
			const Point& p3 = base_mesh().points()[tri[2]];
			const Point& q1 = deformed_mesh().points()[tri[0]];
			const Point& q2 = deformed_mesh().points()[tri[1]];
			const Point& q3 = deformed_mesh().points()[tri[2]];
			Affine tr = affine_triangle_mapping(p1, p2, p3, q1, q2, q3);
			tr.translation() = deformed_marker_position(marker);
			return tr;
		}
	}

	/**
	 * Returns the affine transform of the deformed mesh at the position of the
	 * marker with the given ID.
	 */
	Affine transformation_at_marker(Int marker_id) const {
		return transformation_at_marker(get_marker(marker_id));
	}

	/**
	 * Updates `marker` after a remesh operation.  This is handled automatically
	 * for all markers managed by this puppet, so the method is only useful for
	 * markers which are not managed by this puppet.
	 */
	void remesh_marker(Marker& inout_marker) const {
		if(is_warper_init()) {
			Int triangle_ind;
			Scalar l1, l2, l3; // barycentric coordinates
			m_base_mesh_geometry->nearest_triangle_barycentric(triangle_ind, l1, l2, l3, inout_marker.base_point());
			Assert(triangle_ind >= 0) << "couldn't remesh marker, this is probably a bug" << raise;
			inout_marker.set_triangle(triangle_ind, l1, l2, l3);
		} else {
			inout_marker.unset_triangle();
		}
	}

	/**
	 * Loops over all the markers currently managed by this puppet, calling
	 * `func(marker)` for each.
	 */
	template<typename Func>
	void loop_markers(const Func& func) const {
		for(auto& iter : m_markers) {
			func(iter.second);
		}
	}

	/**
	 * Loops over all the markers currently managed by this puppet, calling
	 * `func(marker_id, marker)` for each.
	 */
	template<typename Func>
	void loop_marker_ids(const Func& func) const {
		for(auto& iter : m_markers) {
			func(iter.first, iter.second);
		}
	}

protected:
	const shared_ptr<PuppetFigureShape<Scalar> >& figure_shape_ptr() const {
		return m_figure;
	}

	void set_figure_shape_ptr(const shared_ptr<PuppetFigureShape<Scalar> >& figure) {
		m_figure = figure;
	}

	virtual void get_mesh_points(PointVector& out_points) const = 0;
	virtual bool iterate_warper_impl() = 0;
	virtual bool is_warper_init() const = 0;
	virtual void init_warper_impl(
		shared_ptr<PuppetMesh<Scalar> >& out_base_mesh,
		shared_ptr<PuppetMeshHandleMap<Scalar> >& out_handle_map
	) = 0;
	virtual void clear_warper_impl() = 0;

	void dirty_mesh() {
		clear_warper_impl();
		m_base_mesh = make_shared<PuppetMesh<Scalar> >();
		m_deformed_mesh.reset();
	}

	void dirty_deformed_mesh() {
		m_is_deformed_mesh_dirty = true;
	}

	void remesh_markers() {
		for(auto& iter : m_markers) {
			Marker& marker = iter.second;
			remesh_marker(marker);
		}
	}

	template<typename Archive>
	void serialize_impl(Archive& ar) {
		// m_base_mesh_geometry is not serialized (since it currently just
		// wraps a mesh and is thus trivial to re-init on load)

		enum {
			MARKER_COUNTER       = 0,
			MARKERS              = 1,
			HAS_MESH             = 2,
			BASE_MESH            = 3,
			HANDLE_MAP           = 4,
			DEFORMED_MESH_POINTS = 5
		};

		ar.object([&](auto& ar) {
			ar("marker_counter",MARKER_COUNTER) & m_marker_counter;
			ar("markers",MARKERS) & m_markers;

			if(!m_base_mesh) {
				m_base_mesh = make_shared<PuppetMesh<Scalar> >();
			}
			bool has_mesh = m_base_mesh->is_init();
			ar("has_mesh",HAS_MESH) & has_mesh;
			if(has_mesh) {
				ar("base_mesh",BASE_MESH) & m_base_mesh;
				ar("handle_map",HANDLE_MAP) & m_handle_map;

				bool has_deformed_mesh = ar.is_input() ? ar.has_field("deformed_mesh_points",DEFORMED_MESH_POINTS) : !m_is_deformed_mesh_dirty;
				if(ar.is_input() && !m_deformed_mesh) {
					shared_ptr<PointVector> deformed_points = make_shared<PointVector>(base_mesh().points());
					m_deformed_mesh = make_shared<PuppetMesh<Scalar> >(
						deformed_points,
						base_mesh().triangles_ptr(),
						base_mesh().point_handles_ptr()
					);
				}
				if(has_deformed_mesh) {
					assert(static_cast<bool>(m_deformed_mesh));
					ar("deformed_mesh_points",DEFORMED_MESH_POINTS) & m_deformed_mesh->points();
					m_is_deformed_mesh_dirty = false;
				} else {
					m_is_deformed_mesh_dirty = true;
				}

				// initialize `m_base_mesh_geometry` if needed
				if(ar.is_input()) {
					m_base_mesh_geometry = make_shared<PuppetMeshGeometryBruteForce<Scalar> >(m_base_mesh);
				}
			}
		});
	}

private:
	shared_ptr<PuppetFigureShape<Scalar> > m_figure;
	shared_ptr<PuppetMesh<Scalar> > m_base_mesh;
	shared_ptr<PuppetMeshHandleMap<Scalar> > m_handle_map;
	
	// stores a pointer to `m_base_mesh`
	shared_ptr<PuppetMeshGeometry<Scalar> > m_base_mesh_geometry;

	// shares triangles and point_handles with m_base_mesh
	bool m_is_deformed_mesh_dirty;
	shared_ptr<PuppetMesh<Scalar> > m_deformed_mesh;

	Int m_marker_counter;
	MarkerMap m_markers;
};

END_OPTIMTOOLS_NAMESPACE
