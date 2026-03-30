#pragma once

/** \file
 * The PuppetWarp deformation operates on a triangular mesh, often generated
 * from a `PuppetFigure`.  The file contains functionality to represent this
 * mesh, as well as utilities to help manage and create these meshes.
 */

#include "optimtools/all.h"

#include "puppet_warp/puppet_figure.h"
#include "puppet_warp/puppet_handle.h"

BEGIN_OPTIMTOOLS_NAMESPACE

// predeclarations

template<typename Scalar>
class BasicPuppetFigureMesher;


/**
 * Represents a triangular mesh to be used in a PuppetWarp algorithm.  This
 * consists of a `PointVector` and a `TriangleVector` giving the geometry
 * of the mesh, and a handle ID for each vertex in the mesh.  The points
 * and vertices are stored as `shared_ptr`s with the intention that they
 * might be shared by multiple meshes (for instance to have multiple
 * shapes with the same mesh topology).
 *
 * As with `PuppetFigure`, methods returning a handleID will return a
 * negative number to indicate no or an invalid handle.
 */
template<typename Scalar>
class PuppetMesh : public serialization::Serializable {
	USING_OPTIMTOOLS_MESH_TYPES(typename,Scalar,2);
	typedef typename Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Matrix;
	typedef typename Eigen::Matrix<Scalar, Eigen::Dynamic, 1> Vector;

public:
	PuppetMesh() { }

	PuppetMesh(
		const shared_ptr<PointVector>& points,
		const shared_ptr<TriangleVector>& triangles,
		const shared_ptr<IntVector>& point_handles
	) :
		m_points(points),
		m_triangles(triangles),
		m_point_handles(point_handles)
	{
		Assert(m_points->size() == m_point_handles->size()) << "`PuppetMesh` must be created with `points` and `point_handles` parameters of the same size" << raise;
	}

	virtual ~PuppetMesh() { }

	bool is_init() const {
		return (bool)m_points;
	}

	Int num_points() const {
		return m_points->size();
	}

	Int num_triangles() const {
		return m_triangles->size();
	}

	const PointVector& points() const {
		return *m_points;
	}

	PointVector& points() {
		return *m_points;
	}

	const TriangleVector& triangles() const {
		return *m_triangles;
	}

	const IntVector& point_handles() const {
		return *m_point_handles;
	}

	const shared_ptr<PointVector>& points_ptr() const {
		return m_points;
	}

	const shared_ptr<TriangleVector>& triangles_ptr() const {
		return m_triangles;
	}

	const shared_ptr<IntVector>& point_handles_ptr() const {
		return m_point_handles;
	}

	Int nearest_handle(const Point& p) const {
		Scalar dist;
		return nearest_deformed_handle(dist, p);
	}

	Int nearest_handle(Scalar& out_dist, const Point& p) const {
		out_dist = std::numeric_limits<Scalar>::max();
		Int min_handle_id = -1;

		// find the nearest handle point
		loop_handle_points([&](const Point& p1, Int handle_id) {
			Scalar dist = (p-p1).matrix().norm();
			if(dist < out_dist) {
				out_dist = dist;
				min_handle_id = handle_id;
			}
		});

		// find the nearest handle segment
		loop_handle_segments([&](const Point& p1, const Point& p2, Int handle_id) {
			Scalar dist = point_segment_distance<Scalar,2>(p, p1, p2);
			if(dist < out_dist) {
				out_dist = dist;
				min_handle_id = handle_id;
			}
		});

		return min_handle_id;
	}

	template<typename Func>
	void loop_handle_points(const Func& func) const {
		loop_handle_point_inds([&](Int point_ind, Int handle_id) {
			const Point& p = points()[point_ind];
			func(p, handle_id);
		});
	}

	template<typename Func>
	void loop_handle_point_inds(const Func& func) const {
		for(Int point_ind = 0; point_ind < num_points(); ++point_ind) {
			Int handle_id = point_handles()[point_ind];
			if(handle_id >= 0) {
				func(point_ind, handle_id);
			}
		}
	}

	template<typename Func>
	void loop_handle_segments(const Func& func) const {
		loop_handle_segment_inds([&](Int i1, Int i2, Int handle_id) {
			const Point& p1 = points()[i1];
			const Point& p2 = points()[i2];
			func(p1, p2, handle_id);
		});
	}

	template<typename Func>
	void loop_handle_segment_inds(const Func& func) const {
		for(Int triangle_ind = 0; triangle_ind < num_triangles(); ++triangle_ind) {
			const Triangle& tri = triangles()[triangle_ind];
			for(Int i = 0; i < 3; ++i) {
				Int i1 = tri[i];
				Int i2 = tri[(i+1)%3];
				Int h1 = point_handles()[i1];
				Int h2 = point_handles()[i2];
				if(h1 >= 0 && h2 >= 0 && h1 == h2) {
					func(i1, i2, h1);
				}
			}
		}
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
			POINTS        = 0,
			TRIANGLES     = 1,
			POINT_HANDLES = 2
		};
		if(!ar.is_input()) {
			Assert(is_init()) << "cannot output-serialize non-initialized `PuppetMesh`" << raise;
		}
		ar.object([&](auto& ar) {
			ar("points",POINTS) & m_points;
			ar("triangles",TRIANGLES) & m_triangles;
			ar("point_handles",POINT_HANDLES) & m_point_handles;
		});
	}

private:
	shared_ptr<PointVector> m_points;
	shared_ptr<TriangleVector> m_triangles;
	shared_ptr<IntVector> m_point_handles;
};


/**
 * A utility class for managing the handles of a `PuppetMesh`.  In particular,
 * it associates each handle ID with an index.  Unlike handle IDs, these
 * indices *are* guaranteed to be consecutive integers starting at `0`.
 */
template<typename Scalar>
class PuppetMeshHandleMap : public serialization::Serializable {
public:
	PuppetMeshHandleMap() { }

	PuppetMeshHandleMap(
		const PuppetFigureShape<Scalar>& figure,
		const IntVector& point_handles
	) {
		init(figure, point_handles);
	}

	virtual ~PuppetMeshHandleMap() { }

	Int num_handles() const {
		return m_num_handles;
	}

	/**
	 * Returns the index associated with the given handle ID.
	 */
	Int handle_index(Int handle_id) const {
		return m_handle_id_to_ind.at(handle_id);
	}

	/**
	 * Returns the ID associated with the given handle index.
	 */
	Int handle_id(Int handle_ind) const {
		return m_handle_ind_to_id[handle_ind];
	}

	/**
	 * Returns a `std::vector<IntVector>` such that the `i`th element in this
	 * vector gives all the vertices in the mesh associated with the handle
	 * with *index* `i`.
	 */
	const std::vector<IntVector>& handle_vertices() const {
		return m_handle_vertices;
	}

	inline virtual void serialize(serialization::Archive& ar) override {
		serialize_impl(ar);
	}

	template<typename Archive>
	inline void serialize(Archive& ar) {
		serialize_impl(ar);
	}

protected:
	void init(
		const PuppetFigureShape<Scalar>& figure,
		const IntVector& point_handles
	) {
		m_num_handles = figure.num_handles();
		m_handle_vertices.resize(m_num_handles);
		m_handle_ind_to_id.resize(m_num_handles);

		// map handle IDs to an index for the handle
		{
			Int handle_ind = 0;
			figure.loop_handle_ids([&](Int handle_id) {
				assert(m_handle_id_to_ind.count(handle_id) == 0);
				m_handle_id_to_ind[handle_id] = handle_ind;
				m_handle_ind_to_id[handle_ind] = handle_id;
				++handle_ind;
			});
		}

		// collect the vertices for each handle
		std::multimap<Int,Int> hind_to_vertex_map; // handle index -> vertices
		for(Int point_ind = 0; point_ind < point_handles.size(); ++point_ind) {
			Int handle_id = point_handles[point_ind];
			if(handle_id >= 0) {
				Int handle_ind = m_handle_id_to_ind.at(handle_id);
				hind_to_vertex_map.insert(std::make_pair(handle_ind,point_ind));
			}
		}

		// store in fixed-size `IntVector`s
		for(Int handle_ind = 0; handle_ind < m_num_handles; ++handle_ind) {
			Int num_handle_verts = hind_to_vertex_map.count(handle_ind);
			assert(num_handle_verts > 0);
			IntVector& handle_verts = m_handle_vertices[handle_ind];
			handle_verts.resize(num_handle_verts);
			auto range = hind_to_vertex_map.equal_range(handle_ind);
			Int i = 0;
			for(auto iter = range.first; iter != range.second; ++iter, ++i) {
				handle_verts[i] = iter->second;
			}
		}
	}

	template<typename Archive>
	void serialize_impl(Archive& ar) {
		enum {
			NUM_HANDLES      = 0,
			HANDLE_VERTICES  = 1,
			HANDLE_ID_TO_IND = 2,
			HANDLE_IND_TO_ID = 3
		};

		ar.object([&](auto& ar) {
			ar("num_handles",NUM_HANDLES) & m_num_handles;
			ar("handle_vertices",HANDLE_VERTICES) & m_handle_vertices;
			ar("handle_id_to_ind",HANDLE_ID_TO_IND) & m_handle_id_to_ind;
			ar("handle_ind_to_id",HANDLE_IND_TO_ID) & m_handle_ind_to_id;
		});
	}

private:
	Int m_num_handles;
	std::vector<IntVector> m_handle_vertices;
	std::map<Int,Int> m_handle_id_to_ind;
	IntVector m_handle_ind_to_id;
};

//---------------------------------------------------------------------------//

namespace internal {

	/**
	 * Utility functions to create a `PuppetMesh` from a `PuppetFigure` using
	 * the `libtriangle` library to do the meshing.
	 */
	template<typename Scalar>
	struct libtriangle_puppet_meshing_utils {
	private:
		USING_OPTIMTOOLS_MESH_TYPES(typename,Scalar,2);

	public:

		/**
		 * Returns a single number representing the linear scale of the given point
		 * set (which will normally be extracted from a `PuppetFigure`.  This used
		 * to determine a default max triangle size when triangulating the figure,
		 * and is currently calculated as the maximum of the width or the height
		 * of the figure's bounding box.
		 */
		static Scalar max_figure_extent(const PointVector& figure_points) {
			Point figure_min_bound = Point::Constant(std::numeric_limits<Scalar>::max());
			Point figure_max_bound = Point::Constant(std::numeric_limits<Scalar>::lowest());
			for(Int point_ind = 0; point_ind < figure_points.size(); ++point_ind) {
				const Point& p = figure_points[point_ind];
				figure_min_bound = figure_min_bound.cwiseMin(p);
				figure_max_bound = figure_max_bound.cwiseMax(p);
			}
			return (figure_max_bound-figure_min_bound).maxCoeff();
		}

		/**
		 * Returns a `libtriangle::MeshingQuality` providing a reasonable
		 * default quality for meshing a figure with the given points.  If
		 * `triangle_size_scale` is greater than zero, it is assumed to give
		 * an approximate hint as the the number of triangles which should span
		 * the meshe's largest dimension (so a square mesh would have roughly
		 * `triangle_size_scale*triangle_size_scale` triangles in total).
		 */
		static libtriangle::MeshingQuality<Scalar> default_quality(
			const PointVector& figure_points,
			const Scalar& triangle_size_scale
		) {
			libtriangle::MeshingQuality<Scalar> quality;

			if(triangle_size_scale > Scalar(0)) {
				Scalar area_scale = triangle_size_scale*triangle_size_scale;
				quality.set_max_triangle_area(area_scale);
			}

			quality.set_max_triangle_angle(0.15); // a bit less than 10 degrees

			return quality;
		}

		/**
		 * Outputs vectors assigning each point and edge in the
		 * given `figure` to their (optionally) associated handle ID.
		 */
		static void collect_figure_handle_ids(
			IntVector& out_point_handles,
			IntVector& out_edge_handles,
			const PuppetFigureShape<Scalar>& figure
		) {
			out_point_handles.resize(figure.num_points());
			out_edge_handles.resize(figure.num_edges());

			out_point_handles.setConstant(-1);
			Int point_ind = 0;
			figure.loop_points([&](const Point& p, const Int& id) {
				out_point_handles[point_ind] = id;
				++point_ind;
			});
			assert(point_ind == out_point_handles.size());

			out_edge_handles.setConstant(-1);
			Int edge_ind = 0;
			figure.loop_segments([&](const Point& p1, Int id1, const Point& p2, Int id2) {
				if(id1 >= 0 && id2 >= 0 && id1 == id2) {
					out_edge_handles[edge_ind] = id1;
				}
				++edge_ind;
			});
			assert(edge_ind == out_edge_handles.size());
		}

		/**
		 * Outputs an `IntVector` giving (optionally) a handle ID for each
		 * vertex in a mesh.  This is created by merging a vector of
		 * pre-existing vertex handle IDs with a vector of edge handle IDs,
		 * which are propagated to the vertices for the edge's endpoints.
		 */
		static void collect_mesh_point_handles(
			IntVector& out_mesh_point_handles,
			const EdgeVector& edges,
			const IntVector& point_handles,
			const IntVector& edge_handles
		) {
			Assert(edges.size() == edge_handles.size()) << "edges and edge_handles must have the same size" << raise;
			Int num_points = point_handles.size();
			Int num_edges = edges.size();

			out_mesh_point_handles.resize(num_points);
			out_mesh_point_handles.setConstant(-1);
			for(Int edge_ind = 0; edge_ind < num_edges; ++edge_ind) {
				Int handle_id = edge_handles[edge_ind];
				if(handle_id >= 0) {
					const Edge& edge = edges[edge_ind];
					out_mesh_point_handles[edge[0]] = handle_id;
					out_mesh_point_handles[edge[1]] = handle_id;
				}
			}
			for(Int point_ind = 0; point_ind < num_points; ++point_ind) {
				Int handle_id = point_handles[point_ind];
				if(handle_id >= 0) {
					out_mesh_point_handles[point_ind] = handle_id;
				}
			}
		}

		/**
		 * Sets `out_mesher` to have a reasonable default initialization for
		 * meshing the given figure.  The optional `triangle_size_scale`
		 * parameter has the same interperation as in the `default_quality`
		 * function.
		 */
		static void init_mesher(
			libtriangle::Mesher<Scalar>& out_mesher,
			const PuppetFigureShape<Scalar>& figure,
			const Scalar& triangle_size_scale=Scalar(-1)
		) {
			PointVector figure_points = figure.points();
			EdgeVector figure_edges = figure.edges();

			libtriangle::MeshingQuality<Scalar> quality = default_quality(figure_points, triangle_size_scale);

			IntVector point_handles, edge_handles;
			collect_figure_handle_ids(point_handles, edge_handles, figure);

			out_mesher = libtriangle::Mesher<Scalar>(quality);
			out_mesher.set_default_point_marker(-1);
			out_mesher.set_default_edge_marker(-1);
			out_mesher.set_create_holes_mode(libtriangle::HoleCreateMode::WINDING_NUMBER_RETRIANGULATE);
			out_mesher.set_points(figure_points);
			out_mesher.set_edges(figure_edges);
			out_mesher.set_point_markers(point_handles);
			out_mesher.set_edge_markers(edge_handles);
		}

		static void solve_mesher(
			libtriangle::Mesher<Scalar>& inout_mesher,
			bool is_add_holes
		) {
			// solve for the mesh
			inout_mesher.solve();
		
			if(is_add_holes) {
				// Remove the triangles for any holes.  We only count edges which are
				// not associated with any handle.
				inout_mesher.create_output_holes_by_winding_number([](Int edge_marker) {
					return edge_marker < 0;
				});
			}
		}

		/**
		 * Initializes `out_mesh` from the output of the given `mesher`.
		 */
		static void init_mesh_from_mesher(
			PuppetMesh<Scalar>& out_mesh,
			const libtriangle::Mesher<Scalar>& mesher
		) {
			Assert(mesher.has_output()) << "`mesher.has_output()` must be `true` for it to be used in `init_mesh_from_mesher`" << raise;

			shared_ptr<PointVector> mesh_points = make_shared<PointVector>();
			shared_ptr<TriangleVector> mesh_triangles = make_shared<TriangleVector>();
			shared_ptr<IntVector> mesh_point_handles = make_shared<IntVector>();

			mesher.get_output_points(*mesh_points);
			mesher.get_output_triangles(*mesh_triangles);

			EdgeVector edges;
			IntVector point_handles, edge_handles;
			mesher.get_output_edges(edges);
			mesher.get_output_point_markers(point_handles);
			mesher.get_output_edge_markers(edge_handles);
			collect_mesh_point_handles(
				*mesh_point_handles,
				edges,
				point_handles,
				edge_handles
			);

			out_mesh = PuppetMesh<Scalar>(mesh_points, mesh_triangles, mesh_point_handles);
		}
	};

} // namespace internal


/**
 * A basic algorithm to create a `PuppetMesh` from a `PuppetFigure`.  This
 * functionality would potentially make sense in a function instead of a
 * class, but the class design was chosen to match that of the more involved
 * (and iterative) `AdaptivePuppetFigureMesher` solver.
 */
template<typename Scalar>
class BasicPuppetFigureMesher {
	USING_OPTIMTOOLS_MESH_TYPES(typename,Scalar,2);
	typedef typename internal::libtriangle_puppet_meshing_utils<Scalar> meshing_utils;

public:
	class Quality {
		friend class BasicPuppetFigureMesher<Scalar>;

	public:
		Quality() :
			m_triangle_size_scale(0.1)
		{
			m_libtriangle_quality.unset_all();
		}

		const Scalar& triangle_size_scale() const {
			return m_triangle_size_scale;
		}

		void set_triangle_size_scale(const Scalar& size_scale) {
			m_triangle_size_scale = size_scale;
		}

		void unset_triangle_size_scale() {
			m_triangle_size_scale = Scalar(-1);
		}

		const Scalar& max_triangle_area() const {
			return m_libtriangle_quality.max_triangle_area();
		}

		void set_max_triangle_area(const Scalar& area) {
			m_libtriangle_quality.set_max_triangle_area(area);
		}

		void unset_max_triangle_area() {
			m_libtriangle_quality.unset_max_triangle_area();
		}

		const Scalar& max_triangle_angle() const {
			return m_libtriangle_quality.max_triangle_angle();
		}

		void set_max_triangle_angle(const Scalar& angle) {
			m_libtriangle_quality.set_max_triangle_angle(angle);
		}

		void unset_max_triangle_angle() {
			m_libtriangle_quality.unset_max_triangle_angle();
		}

		void set_minimal_meshing() {
			unset_triangle_size_scale();
			m_libtriangle_quality.unset_all();
		}

	private:
		libtriangle::MeshingQuality<Scalar> m_libtriangle_quality;
		Scalar m_triangle_size_scale;
	};

public:
	BasicPuppetFigureMesher(const shared_ptr<PuppetFigureShape<Scalar> >& figure) :
		m_figure(figure)
	{ }

	const Quality& quality() const {
		return m_quality;
	}

	void set_quality(const Quality& quality) {
		m_quality = quality;
	}

	void set_minimal_meshing() {
		m_quality.set_minimal_meshing();
	}

	const shared_ptr<PuppetMesh<Scalar> >& mesh() const {
		Assert(static_cast<bool>(m_mesh)) << "cannot call BasicPuppetFigureMesher.mesh() until the mesh has been solved for" << raise;
		return m_mesh;
	}

	const shared_ptr<PuppetMesh<Scalar> >& solve() {
		while(iterate());
		return mesh();
	}

	bool iterate() {
		libtriangle::Mesher<Scalar> mesher;
		meshing_utils::init_mesher(mesher, *m_figure, m_quality.m_triangle_size_scale);

		libtriangle::MeshingQuality<Scalar> libtriangle_quality = mesher.quality();
		libtriangle_quality.set_from(m_quality.m_libtriangle_quality);
		mesher.set_quality(libtriangle_quality);

		meshing_utils::solve_mesher(mesher, m_figure->num_outlines(true) > 0);

		m_mesh = make_shared<PuppetMesh<Scalar> >();
		meshing_utils::init_mesh_from_mesher(*m_mesh, mesher);

		return false;
	}

private:
	shared_ptr<PuppetFigureShape<Scalar> > m_figure;
	Quality m_quality;

	shared_ptr<PuppetMesh<Scalar> > m_mesh;
};


/**
 * Creates an adaptive-resolution `PuppetMesh` from a `PuppetFigure` in
 * situations where linearblend skinning weights are also computed for
 * the mesh.  The resulting mesh will have large triangles in regions where
 * the skinning weights are approximately constant, and small triangles
 * in regions where the skinning weights vary quickly.
 *
 * The algorithm used starts with a very coarse mesh and iteratively refines
 * it, so this class can also be used as a tool for impementing a multi-scale
 * skinning weights solver.
 *
 * AdobePatentID="5675"
 */
template<typename Scalar>
class AdaptivePuppetFigureMesher {
	USING_OPTIMTOOLS_MESH_TYPES(typename,Scalar,2);
	typedef Eigen::Matrix<double, Eigen::Dynamic, 1> Vector;
	typedef typename Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Matrix;

	/**
	 * This solver uses a function pointer to compute the skinning weights.
	 * This typedef gives the signature of this function pointer.
	 */
	typedef std::function<void (
		Matrix& out_weights,
		const PuppetFigureShape<Scalar>& figure,
		const PuppetMesh<Scalar>& mesh,
		const PuppetMeshHandleMap<Scalar>& handle_map,
		const AdaptivePuppetFigureMesher<Scalar>& mesher
	)> SolveSkinningWeightsFunction;

public:

	/**
	 * Represents the parameters impacting the quality of the meshing.
	 */
	class Quality : public serialization::Serializable {
	public:
		Quality() :
			m_max_num_triangles_hint(-1),
			m_max_triangle_weight_change(0.2),
			m_max_triangle_area(-1),
			m_min_triangle_area_hint(0)
		{ }

		bool has_max_num_triangles_hint() const {
			return m_max_num_triangles_hint >= 1;
		}

		bool has_max_triangle_area() const {
			return m_max_triangle_area > Scalar(0);
		}

		bool has_min_triangle_area_hint() const {
			return m_min_triangle_area_hint > Scalar(0);
		}

		Int max_num_triangles_hint() const {
			assert(has_max_num_triangles_hint());
			return m_max_num_triangles_hint;
		}

		Int& max_num_triangles_hint() {
			return m_max_num_triangles_hint;
		}

		const Scalar& max_triangle_weight_change() const {
			return m_max_triangle_weight_change;
		}

		Scalar& max_triangle_weight_change() {
			return m_max_triangle_weight_change;
		}

		const Scalar& max_triangle_area() const {
			assert(has_max_triangle_area());
			return m_max_triangle_area;
		}

		Scalar& max_triangle_area() {
			return m_max_triangle_area;
		}

		const Scalar& min_triangle_area_hint() const {
			assert(has_min_triangle_area_hint());
			return m_min_triangle_area_hint;
		}

		Scalar& min_triangle_area_hint() {
			return m_min_triangle_area_hint;
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
				MAX_NUM_TRIANGLES_HINT     = 0,
				MAX_TRIANGLE_WEIGHT_CHANGE = 1,
				MAX_TRIANGLE_AREA          = 2,
				MIN_TRIANGLE_AREA_HINT     = 3
			};
			ar.object([&](auto& ar) {
				ar("max_num_triangles_hint",MAX_NUM_TRIANGLES_HINT) & m_max_num_triangles_hint;
				ar("max_triangle_weight_change",MAX_TRIANGLE_WEIGHT_CHANGE) & m_max_triangle_weight_change;
				ar("max_triangle_area",MAX_TRIANGLE_AREA) & m_max_triangle_area;
				ar("min_triangle_area_hint",MIN_TRIANGLE_AREA_HINT) & m_min_triangle_area_hint;
			});
		};

	private:
		Int m_max_num_triangles_hint;
		Scalar m_max_triangle_weight_change;
		Scalar m_max_triangle_area;
		Scalar m_min_triangle_area_hint;
	};

public:
	AdaptivePuppetFigureMesher(
		const shared_ptr<PuppetFigureShape<Scalar> >& figure,
		const SolveSkinningWeightsFunction& skinning_function
	) :
		m_figure(figure),
		m_skinning_function(skinning_function),
		m_is_mesher_init(false),
		m_is_iterating(true)
	{ }

	const Quality& quality() const {
		return m_quality;
	}

	void set_quality(const Quality& quality) {
		m_quality = quality;
	}

	const shared_ptr<PuppetMesh<Scalar> >& mesh() const {
		Assert(static_cast<bool>(m_mesh)) << "cannot call AdaptivePuppetFigureMesher.mesh() until the mesh has been solved for" << raise;
		return m_mesh;
	}

	const shared_ptr<PuppetMeshHandleMap<Scalar> > handle_map() const {
		Assert(static_cast<bool>(m_handle_map)) << "cannot call AdaptivePuppetFigureMesher.handle_map() until the mesh has been solved for" << raise;
		return m_handle_map;
	}

	const Matrix& skinning_weights() const {
		return m_skinning_weights;
	}

	Matrix& skinning_weights() {
		return m_skinning_weights;
	}

	const shared_ptr<PuppetMesh<Scalar> >& solve() {
		while(iterate());
		return mesh();
	}

	bool iterate() {
		typedef typename internal::libtriangle_puppet_meshing_utils<Scalar> utils;

		if(!m_is_mesher_init) {
			utils::init_mesher(m_mesher, *m_figure, Scalar(-1));
			if(m_quality.has_max_triangle_area()) {
				libtriangle::MeshingQuality<Scalar> libtriangle_quality = m_mesher.quality();
				libtriangle_quality.set_max_triangle_area(m_quality.max_triangle_area());
				m_mesher.set_quality(libtriangle_quality);
			}
			m_is_mesher_init = true;
		}

		utils::solve_mesher(m_mesher, m_figure->num_outlines(true) > 0);

		Assert(m_mesher.num_output_triangles() > m_mesher.num_triangles()) << "infinite loop in mesh refinement, this is a bug" << raise;
		m_mesh = make_shared<PuppetMesh<Scalar> >();
		utils::init_mesh_from_mesher(*m_mesh, m_mesher);

		// create the handle map
		m_handle_map = make_shared<PuppetMeshHandleMap<Scalar> >(*m_figure, mesh()->point_handles());
		Int num_handles = m_figure->num_handles();

		// solve for the skinning weights
		if(m_mesher.num_point_attributes() > 0) {
			assert(m_mesher.num_point_attributes() == m_handle_map->num_handles());
			m_mesher.get_output_point_attributes(m_skinning_weights);
			m_skinning_weights.transposeInPlace();
		}
		assert(m_skinning_weights.size() == 0 || (m_skinning_weights.rows() == m_mesh->num_points() && m_skinning_weights.cols() == num_handles)); 
		m_skinning_function(m_skinning_weights, *m_figure, *m_mesh, *m_handle_map, *this);

		if(num_handles <= 1) {
			// For zero or 1 handles, the motion is affine, so the coarsest
			// possible triangulation is sufficient and the iteration can
			// be terminated now.  This is early termination is done after
			// the call to m_skinning_function so we can be sure that
			// m_skinning_function is call at least once.
			m_is_iterating = false;
		} else {
			// refine the mesh if necessary
			const PointVector& points = mesh()->points();
			const TriangleVector& triangles = mesh()->triangles();
			Vector triangle_areas;
			bool is_need_refinement = subdivide_triangle_areas(triangle_areas, points, triangles, m_skinning_weights);
		
			if(is_need_refinement) {
				m_mesher.refine_output();
				m_mesher.set_triangle_areas(triangle_areas);

				m_skinning_weights.transposeInPlace();
				m_mesher.set_point_attributes(m_skinning_weights);
				// we can skip transposing back, since m_skinning_weights will be
				// overwritten in the next iteration after we remesh and before it's
				// used anywhere
			} else {
				m_is_iterating = false;
			}
		}
		return m_is_iterating;
	}

protected:

	bool subdivide_triangle_areas(
		Vector& out_triangle_areas,
		const PointVector& points,
		const TriangleVector& triangles,
		const Matrix& skinning_weights
	) const {
		bool is_need_refinement;

		if(
			   quality().has_max_num_triangles_hint()
			&& triangles.size() >= quality().max_num_triangles_hint()
		) {
			is_need_refinement = false;
		} else {
			is_need_refinement = false;
			out_triangle_areas.resize(triangles.size());
			out_triangle_areas.setConstant(std::numeric_limits<Scalar>::max());

			for(Int triangle_ind = 0; triangle_ind < triangles.size(); ++triangle_ind) {
				const Triangle& triangle = triangles[triangle_ind];
			
				// calculate the maximum change in any of the skinning weights
				// over the edges of the triangle.
				Scalar triangle_weight_change = 0;
				auto w1 = skinning_weights.row(triangle[0]);
				auto w2 = skinning_weights.row(triangle[1]);
				auto w3 = skinning_weights.row(triangle[2]);
				triangle_weight_change = std::max(triangle_weight_change, (w2-w1).cwiseAbs().maxCoeff());
				triangle_weight_change = std::max(triangle_weight_change, (w3-w2).cwiseAbs().maxCoeff());
				triangle_weight_change = std::max(triangle_weight_change, (w3-w1).cwiseAbs().maxCoeff());

				// if the skinning weights change enough, we need to subdivide (unless
				// the triangle is already too small).
				if(triangle_weight_change > quality().max_triangle_weight_change()) {
					const Point& p1 = points[triangle[0]];
					const Point& p2 = points[triangle[1]];
					const Point& p3 = points[triangle[2]];
					Scalar old_area = triangle_area(p1, p2, p3);
					assert(old_area > Scalar(1e-10)); // if we have a triangle this small we probably have a mesh with coincident handles

					if(
						  !quality().has_min_triangle_area_hint()
						|| old_area > quality().min_triangle_area_hint()
					) {
						// the triangle is large enough, so subdivide it
						Scalar weight_ratio = triangle_weight_change / quality().max_triangle_weight_change();
						Scalar area_ratio = weight_ratio * weight_ratio;
						area_ratio = std::max(area_ratio, Scalar(2));
						area_ratio = std::min(area_ratio, Scalar(16));
						out_triangle_areas[triangle_ind] = old_area / area_ratio;
						is_need_refinement = true;
					}
				}
			}
		}

		return is_need_refinement;
	}

private:
	shared_ptr<PuppetFigureShape<Scalar> > m_figure;
	Quality m_quality;

	SolveSkinningWeightsFunction m_skinning_function;
	libtriangle::Mesher<Scalar> m_mesher;
	bool m_is_mesher_init;
	bool m_is_iterating;

	shared_ptr<PuppetMesh<Scalar> > m_mesh;
	shared_ptr<PuppetMeshHandleMap<Scalar> > m_handle_map;
	Matrix m_skinning_weights;
};

END_OPTIMTOOLS_NAMESPACE
