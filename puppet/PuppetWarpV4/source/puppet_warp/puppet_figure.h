#pragma once

/** \file
 * A puppet's *figure* describes the aspects of a puppet which are independent
 * of how it is meshed or deformed.  This consists of the puppet's silhouette
 * and information about the location and type of handles in the puppet.
 */

#include <set>
#include <map>

#include "optimtools/all.h"
#include "optimtools_extras/geometry/all.h"
#include "optimtools_extras/mesh/all.h"

#include "puppet_warp/puppet_handle.h"


BEGIN_OPTIMTOOLS_NAMESPACE

/**
 * Retresents a PSLG with a subset of the points and edges marked as handles,
 * along with various utility methods.  This factors out the information about
 * a puppet's figure which does not depend on on the actual information stored
 * at each of the handles, but only on the pure geometry of the puppet.  An
 * actual puppet's figure will be a specialization of `PuppetFigure`, which is
 * a subclass of this class.
 */
template<typename Scalar>
class PuppetFigureShape {
	USING_OPTIMTOOLS_MESH_TYPES(typename,Scalar,2);

	typedef typename PSLG<Scalar,Int>::TmpPointRef TmpPointRef;

public:
	PuppetFigureShape() :
		m_pslg(-1),
		m_handle_counter(0)
	{ }

	PuppetFigureShape(const PSLG<Scalar,Int>& pslg) :
		m_pslg(pslg),
		m_handle_counter(0)
	{
		Assert(m_pslg.default_vertex_data() == -1) << "Cannot create a PuppetFigure from a PSLG for which `pslg.default_vertex_data() != -1`." << raise;
	}

	virtual ~PuppetFigureShape() { }

	virtual PuppetFigureShape<Scalar>* clone() const = 0;

	virtual Int num_handles() const = 0;

	virtual bool has_handle(Int handle_id) const = 0;

	Int num_points() const {
		return m_pslg.num_points();
	}

	Int num_edges() const {
		return m_pslg.num_edges();
	}

	Int num_outlines() const {
		return m_pslg.num_outlines();
	}

	Int num_outlines(bool is_hole) const {
		return m_pslg.num_outlines(is_hole);
	}

	PointVector points() const {
		return m_pslg.points();
	}

	EdgeVector edges() const {
		return m_pslg.edges();
	}

	void add_outline(const PointVector& points, bool is_hole=false) {
		m_pslg.add_outline(points, is_hole);
	}

	std::vector<PointVector> get_outline_point_loops(bool is_hole) const {
		return m_pslg.get_outline_point_loops(is_hole);
	}

	virtual Int add_point_handle(const Point& p, const Scalar& threshold_dist) {
		Int handle_id = m_handle_counter;
		assert(!has_handle(handle_id));

		// find the nearest handle point as well as the nearest overall point.
		// These two points may be equal.
		TmpPointRef nearest_p_ref;
		Scalar min_dist;
		Int nearest_id;
		Scalar min_handle_dist;
		nearest_point_and_handle(nearest_p_ref, min_dist, nearest_id, min_handle_dist, p);

		// add a new handle (or not) depending on if there is something already at p
		if(min_handle_dist <= threshold_dist) {
			// there's already a handle there, so we can't add a new one
			return -1;
		} else {
			if(min_dist <= threshold_dist) {
				// there's already a point there, so mark it as belonging to the handle
				m_pslg.set_point_data(nearest_p_ref, handle_id);
			} else {
				// add a new handle point
				m_pslg.add_point(p, handle_id);
			}

			m_handle_ids.insert(handle_id);
			++m_handle_counter;
			return handle_id;
		}
	}

	virtual bool add_segment(
		const Point& p1,
		const Point& p2,
		const Scalar& threshold_dist,
		const Scalar& pslg_threshold_dist
	) {
		return add_segment_helper(p1, p2, threshold_dist, pslg_threshold_dist, -1);
	}

	virtual Int add_segment_handle(
		const Point& p1,
		const Point& p2,
		const Scalar& threshold_dist,
		const Scalar& pslg_threshold_dist
	) {
		Int handle_id = m_handle_counter;
		assert(!has_handle(handle_id));

		bool is_able_to_add = add_segment_helper(p1, p2, threshold_dist, pslg_threshold_dist, handle_id);
		if(is_able_to_add) {
			m_handle_ids.insert(handle_id);
			++m_handle_counter;
			return handle_id;
		} else {
			// we can't add a new handle
			return -1;
		}
	}
	
	Int add_segment_handle(const Point& p1, const Point& p2, const Scalar& threshold_dist) {
		return add_segment_handle(p1, p2, threshold_dist, threshold_dist);
	}

	virtual void remove_handle(Int handle_id) {
		Assert(has_handle(handle_id)) << "attempt to remove a non-existant handle with ID " << handle_id << raise;
		m_pslg.loop_outline_point_tmprefs([&](const TmpPointRef& p_ref) {
			Point p;
			Int id;
			m_pslg.get_point(p, id, p_ref);
			if(id == handle_id) {
				m_pslg.set_point_data(p_ref, -1);
			}
		});
		m_pslg.remove_nonoutline_points_if([&](const Point& p, const Int& id) {
			return id == handle_id;
		});
		m_handle_ids.erase(handle_id);
	}

	template<typename Func>
	void loop_segments(const Func& func) const {
		m_pslg.loop_segments(func);
	}

	template<typename Func>
	void loop_points(const Func& func) const {
		m_pslg.loop_points(func);
	}

	template<typename Func>
	void loop_handle_ids(const Func& func) const {
		for(Int handle_id : m_handle_ids) {
			func(handle_id);
		}
	}

	Int nearest_handle(const Point& p) const {
		Scalar dist;
		return nearest_handle(dist, p);
	}

	Int nearest_handle(Scalar& out_dist, const Point& p) const {
		TmpPointRef nearest_p_ref;
		Scalar min_dist;
		Int nearest_id;
		Scalar min_handle_dist;
		nearest_point_and_handle(nearest_p_ref, min_dist, nearest_id, min_handle_dist, p);

		out_dist = min_handle_dist;
		return nearest_id;
	}

	Int nearest_handle(Scalar& out_dist, const Point& p1, const Point& p2) const {
		TmpPointRef nearest_p_ref;
		Scalar min_dist;
		Int nearest_id;
		Scalar min_handle_dist;
		nearest_point_and_handle(nearest_p_ref, min_dist, nearest_id, min_handle_dist, p1, p2);

		out_dist = min_handle_dist;
		return nearest_id;
	}

	Scalar nearest_pslg_distance(const Point& p1, const Point& p2) const {
		Scalar min_dist = std::numeric_limits<Scalar>::max();
		m_pslg.loop_points([&](const Point& q, Int id) {
			Scalar dist = point_segment_distance<Scalar,2>(q, p1, p2);
			if(dist < min_dist) {
				min_dist = dist;
			}
		});
		m_pslg.loop_segments([&](const Point& q1, const Int& id1, const Point& q2, const Int& id2) {
			Scalar dist = segment_segment_distance<Scalar,2>(q1, q2, p1, p2);
			if(dist < min_dist) {
				min_dist = dist;
			}
		});
		return min_dist;
	}

	bool is_point_inside(const Point& p) const {
		return m_pslg.is_point_inside(p);
	}

	Point project_to_nearest_outline(const Point& p) const {
		return m_pslg.project_to_nearest_outline(p);
	}

protected:
	PSLG<Scalar,Int>& pslg() {
		return m_pslg;
	}

	Int& handle_id_counter() {
		return m_handle_counter;
	}

	virtual bool add_segment_helper(
		const Point& p1,
		const Point& p2,
		const Scalar& threshold_dist,
		const Scalar& pslg_threshold_dist,
		Int handle_id
	) {
		// find the nearest handle point as well as the nearest overall point.
		// These two points may be equal.
		TmpPointRef nearest_p_ref;
		Scalar min_dist;
		Int nearest_id;
		Scalar min_handle_dist;
		nearest_point_and_handle(nearest_p_ref, min_dist, nearest_id, min_handle_dist, p1, p2);

		if(min_handle_dist <= threshold_dist) {
			// there's already a handle there, so we can't add a new one
			return false;
		} else {
			// we need to check if the segment lies on the PSLG
			// technically, we could go and modify the PSLG to match the added handle,
			// but for now it's easier just to not add the handle in the first place.
			min_dist = nearest_pslg_distance(p1, p2);
			if(min_dist <= pslg_threshold_dist) {
				return false;
			}

			// add a new segment
			m_pslg.add_segment(p1, p2, handle_id);
			return true;
		}
	}

	void nearest_point_and_handle(
		TmpPointRef& out_nearest_p_ref,
		Scalar& out_min_dist,
		Int& out_nearest_id,
		Scalar& out_min_handle_dist,
		const Point& p
	) const {
		nearest_point_and_handle_helper(
			out_nearest_p_ref,
			out_min_dist,
			out_nearest_id,
			out_min_handle_dist,
			[&](const Point& q) {
				return (p-q).matrix().norm();
			},
			[&](const Point& q1, const Point& q2) {
				return point_segment_distance<Scalar,2>(p, q1, q2);
			}
		);
	}

	void nearest_point_and_handle(
		TmpPointRef& out_nearest_p_ref,
		Scalar& out_min_dist,
		Int& out_nearest_id,
		Scalar& out_min_handle_dist,
		const Point& p1,
		const Point& p2
	) const {
		nearest_point_and_handle_helper(
			out_nearest_p_ref,
			out_min_dist,
			out_nearest_id,
			out_min_handle_dist,
			[&](const Point& q) {
				return point_segment_distance<Scalar,2>(q, p1, p2);
			},
			[&](const Point& q1, const Point& q2) {
				return segment_segment_distance<Scalar,2>(p1, p2, q1, q2);
			}
		);
	}

	template<typename PointDistFunc, typename SegDistFunc>
	void nearest_point_and_handle_helper(
		TmpPointRef& out_nearest_p_ref,
		Scalar& out_min_dist,
		Int& out_nearest_id,
		Scalar& out_min_handle_dist,
		const PointDistFunc& point_dist_func,
		const SegDistFunc& seg_dist_func
	) const {
		TmpPointRef nearest_p_ref;
		Scalar min_dist = std::numeric_limits<Scalar>::max();
		Int nearest_id = -1;
		Scalar min_handle_dist = std::numeric_limits<Scalar>::max();
		m_pslg.loop_point_tmprefs([&](const TmpPointRef& p_ref) {
			Point p;
			Int id;
			m_pslg.get_point(p, id, p_ref);
			Scalar dist = point_dist_func(p);
			if(dist < min_dist) {
				nearest_p_ref = p_ref;
				min_dist = dist;
			}
			if(id >= 0 && dist < min_handle_dist) {
				nearest_id = id;
				min_handle_dist = dist;
			}
		});
		m_pslg.loop_segments([&](const Point& p1, const Int& id1, const Point& p2, const Int& id2) {
			if(id1 >= 0 && id2 >= 0 && id1 == id2) {
				Scalar dist = seg_dist_func(p1, p2);
				if(dist < min_handle_dist) {
					nearest_id = id1;
					min_handle_dist = dist;
				}
			}
		});

		out_nearest_p_ref = nearest_p_ref;
		out_min_dist = min_dist;
		out_nearest_id = nearest_id;
		out_min_handle_dist = min_handle_dist;
	}

	template<typename HandleMap>
	void set_handle_map(const HandleMap& handles) {
		m_handle_ids.clear();
		for(const auto& kv : handles) {
			m_handle_ids.insert(kv.first);
		}
	}

private:
	PSLG<Scalar,Int> m_pslg;
	Int m_handle_counter;
	std::set<Int> m_handle_ids;
};


/**
 * A `PuppetFigure` describes the shape of and handles in a puppet.  The
 * shape is described by a PSLG with markers assigning each vertex in the PSLG
 * (optionally) to a single handle.
 *
 * For any method in this class returning a handle ID, a negative value is
 * used to indicate no or an invalid handle.
 */
template<typename Scalar, typename Handle>
class PuppetFigure : public PuppetFigureShape<Scalar>, serialization::Serializable {
	USING_OPTIMTOOLS_MESH_TYPES(typename,Scalar,2);

	typedef typename PSLG<Scalar,Int>::TmpPointRef TmpPointRef;

public:
	PuppetFigure() :
		PuppetFigureShape<Scalar>()
	{ }

	PuppetFigure(const PSLG<Scalar,Int>& pslg) :
		PuppetFigureShape<Scalar>(pslg)
	{ }

	virtual ~PuppetFigure() { }

	virtual PuppetFigure<Scalar,Handle>* clone() const override {
		return new PuppetFigure<Scalar,Handle>(*this);
	}

	virtual Int num_handles() const override {
		return m_handles.size();
	}

	virtual bool has_handle(Int handle_id) const override {
		return m_handles.count(handle_id) > 0;
	}

	const Handle& handle(Int handle_id) const {
		return *handle_ptr(handle_id);
	}

	const shared_ptr<Handle>& handle_ptr(Int handle_id) const {
		Assert(has_handle(handle_id)) << "attempt to access non-existant handle with ID " << handle_id << raise;
		return m_handles.at(handle_id);
	}

	template<typename Func>
	void loop_handles(const Func& func) const {
		for(const auto& kv : m_handles) {
			Int handle_id = kv.first;
			const auto& handle = kv.second;
			assert(handle_id == handle->id());
			func(*handle);
		}
	}

	virtual Int add_point_handle(const Point& p, const Scalar& threshold_dist) override {
		Int handle_id = PuppetFigureShape<Scalar>::add_point_handle(p, threshold_dist);
		if(handle_id >= 0) {
			m_handles[handle_id] = make_shared<Handle>(handle_id);
		}
		return handle_id;
	}

	virtual Int add_segment_handle(
		const Point& p1,
		const Point& p2,
		const Scalar& threshold_dist,
		const Scalar& pslg_threshold_dist
	) override {
		Int handle_id = PuppetFigureShape<Scalar>::add_segment_handle(p1, p2, threshold_dist, pslg_threshold_dist);
		if(handle_id >= 0) {
			m_handles[handle_id] = make_shared<Handle>(handle_id);
		}
		return handle_id;
	}

	virtual void remove_handle(Int handle_id) override {
		PuppetFigureShape<Scalar>::remove_handle(handle_id);
		m_handles.erase(handle_id);
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
			PSLG              = 0,
			HANDLE_ID_COUNTER = 1,
			HANDLES           = 2
		};

		ar.object([&](auto& ar) {
			ar("pslg",PSLG) & this->pslg();
			ar("handle_id_counter",HANDLE_ID_COUNTER) & this->handle_id_counter();
			ar("handles",HANDLES) & m_handles;
		});

		if(ar.is_input()) {
			this->set_handle_map(m_handles);
		}
	}

private:
	std::map<Int, shared_ptr<Handle> > m_handles;
};

END_OPTIMTOOLS_NAMESPACE
