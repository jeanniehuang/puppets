#pragma once

/** \file
 * Defined a simple puppet which minimizes a per-triangle ARAP energy.
 * Primarily intended as a simple example, so it's probably not all that useful
 * in practice.
 */

#include "optimtools/all.h"

#include "puppet_warp/puppet_base.h"

BEGIN_OPTIMTOOLS_NAMESPACE


/**
 * An abstract base class to make it easier to implement basic puppets backed
 * by an optimtools-style full-mesh deformation algorithm.
 */
template<typename Scalar, class Warper>
class FullMeshWarperPuppet : public Puppet<Scalar> {
	USING_OPTIMTOOLS_MESH_TYPES(typename,Scalar,2);

	typedef typename Puppet<Scalar>::HandleDragger HandleDragger;

public:
	typedef Eigen::Matrix<Scalar,Eigen::Dynamic,1> Vector;
	typedef PuppetFigure<Scalar,PuppetHandle> Figure;
	typedef typename BasicPuppetFigureMesher<Scalar>::Quality MeshingQuality;

	enum class HandleType {
		PIN,
		CONSTRAINT
	};

	class HandleInfo  {
	public:
		HandleInfo() = default;

		HandleInfo(
			Int id,
			HandleType handle_type,
			IntVector verts
		) :
			m_id(id),
			m_handle_type(handle_type),
			m_verts(std::move(verts))
		{ }

		Int id() const {
			return m_id;
		}

		HandleType handle_type() const {
			return m_handle_type;
		}

		const IntVector& verts() const {
			return m_verts;
		}

	private:
		Int m_id{-1};
		HandleType m_handle_type{HandleType::PIN};
		IntVector m_verts;
	};

public:
	FullMeshWarperPuppet() { }

	FullMeshWarperPuppet(const Figure& figure) {
		shared_ptr<Figure> figure_copy = make_shared<Figure>(figure);
		Puppet<Scalar>::init(figure_copy);
		m_figure = figure_copy; 
	}

	virtual void move_dragger(const HandleDragger& dragger, const Point& p) override {
		assert(this->is_warper_init());
		assert(this->is_valid_dragger(dragger));
		Int handle_ind = this->handle_map().handle_index(dragger.handle_id());
		m_warper->set_handle_point(handle_ind, p);
	}

	virtual void set_handle_point(Int handle_id, const Point& p) {
		Assert(this->is_warper_init()) << "cannot call `set_handle_point()` before the warper is initialized" << raise;
		Int handle_ind = this->handle_map().handle_index(handle_id);
		m_warper->set_handle_point(handle_ind, p);
	}

	virtual Point handle_point(Int handle_id){
		return m_warper->handle_point(handle_id);
	}

	const Figure& figure() const {
		return *m_figure;
	}

	const MeshingQuality& meshing_quality() const {
		return m_meshing_quality;
	}

	MeshingQuality& meshing_quality() {
		return m_meshing_quality;
	}
	
	const shared_ptr<const Warper>& warper() const {
		return m_warper;
	}
	
	shared_ptr<Warper>& warper() {
		return m_warper;
	}

	Vector extrapolate_handle_values(const Vector& handle_values) const {
		Assert(this->is_warper_init()) << "cannot call `vertex_depths_from_handle_depths` unless you have an initialized mesh and warper" << raise;
		Assert(handle_values.size() == this->num_handles()) << "when calling `extrapolate_handle_values`, `handle_values.size()` must be equal to the number of handles in the puppet" << raise;
		
		Int num_handle_verts = 0;
		loop_handle_verts([&](Int handle_ind, Int vert_ind) { num_handle_verts += 1; });
		Vector expanded_handle_values(num_handle_verts);
		{
			Int constraint_ind = 0;
			loop_handle_verts([&](Int handle_ind, Int vert_ind) {
				expanded_handle_values[constraint_ind] = handle_values[handle_ind];
				constraint_ind += 1;
			});
		}
		
		Int num_mesh_verts = this->base_mesh().num_points();
		Vector vertex_values;
		m_vertex_value_solver->solve_to(vertex_values, Vector::Zero(num_mesh_verts), expanded_handle_values);
		return vertex_values;
	}

protected:
	virtual shared_ptr<Warper> init_warper_from_mesh(
		const PointVector& points,
		const TriangleVector& triangles,
		const std::vector<HandleInfo>& handle_infos
	) const = 0;

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
		return (bool)m_warper;
	}

	virtual void init_warper_impl(
		shared_ptr<PuppetMesh<Scalar> >& out_base_mesh,
		shared_ptr<PuppetMeshHandleMap<Scalar> >& out_handle_map
	) override {
		Assert(this->figure().num_handles() >= 1) << "cannot initialize an ARAP warper with zero handles" << raise;
		BasicPuppetFigureMesher<Scalar> mesher(figure_ptr());
		mesher.set_quality(m_meshing_quality);
		out_base_mesh = mesher.solve();
		out_handle_map = make_shared<PuppetMeshHandleMap<Scalar> >(figure(), out_base_mesh->point_handles());
		
		const PointVector& points = out_base_mesh->points();
		const TriangleVector& triangles = out_base_mesh->triangles();

		// init `m_handle_infos`
		Int num_handles = out_handle_map->num_handles();
		m_handle_infos.resize(num_handles);
		const std::vector<IntVector>& handle_vert_sets = out_handle_map->handle_vertices();
		assert(handle_vert_sets.size() == num_handles);
		for(Int handle_ind = 0; handle_ind < num_handles; ++handle_ind) {
			const IntVector& handle_verts = handle_vert_sets.at(handle_ind);
			Int num_handle_verts = handle_verts.size();
			m_handle_infos.at(handle_ind) = HandleInfo(
				out_handle_map->handle_id(handle_ind),
				num_handle_verts == 1 ? HandleType::PIN : HandleType::CONSTRAINT,
				handle_verts
			);
		}

		m_warper = init_warper_from_mesh(points, triangles, m_handle_infos);

		// initialize `m_vertex_value_solver`
		{
			Eigen::SparseMatrix<Scalar> L;
			cotangent_laplacian<Scalar,2>::mesh_laplacian(L, points, triangles);
			L = L.transpose() * L; // minimize squared Laplacian energy

			Int num_handle_verts = 0;
			loop_handle_verts([&](Int handle_ind, Int vert_ind) { num_handle_verts += 1; });
			Eigen::SparseMatrix<Scalar> Aeq;
			build_matrix<Eigen::SparseMatrix<Scalar> >(Aeq, num_handle_verts, points.size(), [&](auto& Aeq) {
				Int constraint_ind = 0;
				loop_handle_verts([&](Int handle_ind, Int vert_ind) {
					Aeq(constraint_ind, vert_ind) += Scalar(1);
					constraint_ind += 1;
				});
			});
			
			m_vertex_value_solver = make_shared<LCQSolver<Eigen::SparseMatrix<Scalar> > >(L, Aeq);
		}
	}

	virtual void clear_warper_impl() override {
		m_warper.reset();
	}

	template<typename Func>
	void loop_handle_verts(const Func& func) const {
		for(Int handle_ind = 0; handle_ind < int_cast<Int>(m_handle_infos.size()); ++handle_ind) {
			const IntVector& handle_verts = m_handle_infos.at(handle_ind).verts();
			for(Int handle_vert_ind = 0; handle_vert_ind < handle_verts.size(); ++handle_vert_ind) {
				Int vert_ind = handle_verts[handle_vert_ind];
				func(handle_ind, vert_ind);
			}
		}
	}

	static IntVector collect_point_handle_verts(const std::vector<HandleInfo>& handle_infos) {
		IntVector handle_verts(handle_infos.size());
		for(Int handle_ind = 0; handle_ind < handle_verts.size(); ++handle_ind) {
			const IntVector& verts = handle_infos.at(handle_ind).verts();
			Assert(verts.size() == 1) << "non-point handles not supported in this subclass of `FullMeshWarperPuppet`" << raise;
			handle_verts[handle_ind] = verts[0];	
		}
		return handle_verts;
	}

private:
	shared_ptr<Figure> m_figure; // points to same figure as in superclass (but has a more specific type)
	MeshingQuality m_meshing_quality;
	shared_ptr<Warper> m_warper;
	std::vector<HandleInfo> m_handle_infos;

	shared_ptr<LCQSolver<Eigen::SparseMatrix<Scalar> > > m_vertex_value_solver;
};


/**
 * Creates a puppet with a deformation backed by a standard local-global
 * ARAP algorithm.
 */
template<typename Scalar>
class AlternatingARAPPuppet : public FullMeshWarperPuppet<Scalar, AlternatingARAPMeshWarper<Scalar,2> > {
	typedef FullMeshWarperPuppet<Scalar, AlternatingARAPMeshWarper<Scalar,2> > Super;
	USING_OPTIMTOOLS_MESH_TYPES(typename,Scalar,2);

public:
	using typename Super::Handle;
	using typename Super::Figure;
	using typename Super::MeshingQuality;
	using typename Super::HandleInfo;

public:
	AlternatingARAPPuppet() { }

	AlternatingARAPPuppet(const Figure& figure) : Super(figure) { }
	
	void reset_solver() {
		this->warper()->reset_solver();
	}

protected:
	virtual shared_ptr<AlternatingARAPMeshWarper<Scalar,2> > init_warper_from_mesh(
		const PointVector& points,
		const TriangleVector& triangles,
		const std::vector<HandleInfo>& handle_infos
	) const {
		return make_shared<AlternatingARAPMeshWarper<Scalar,2> >(
			points,
			triangles,
			this->collect_point_handle_verts(handle_infos)
		);
	}
};


/**
 * Creates a puppet with a deformation backed by a standard local-global
 * ARAP algorithm with a smoothing energy defined on the deformation gradients
 * of adjecent triangles.
 */
template<typename Scalar>
class AlternatingSmoothedARAPPuppet : public FullMeshWarperPuppet<Scalar, AlternatingSmoothedARAPMeshWarper<Scalar,2> > {
	typedef FullMeshWarperPuppet<Scalar, AlternatingSmoothedARAPMeshWarper<Scalar,2> > Super;
	USING_OPTIMTOOLS_MESH_TYPES(typename,Scalar,2);

public:
	using typename Super::Handle;
	using typename Super::Figure;
	using typename Super::MeshingQuality;
	using typename Super::HandleInfo;

public:
	AlternatingSmoothedARAPPuppet() = default;

	AlternatingSmoothedARAPPuppet(const Figure& figure, const Scalar& smoothing_weight) :
		Super(figure),
		m_smoothing_weight(smoothing_weight)
	{ }
	
	void reset_solver() {
		this->warper()->reset_solver();
	}

protected:
	virtual shared_ptr<AlternatingSmoothedARAPMeshWarper<Scalar,2> > init_warper_from_mesh(
		const PointVector& points,
		const TriangleVector& triangles,
		const std::vector<HandleInfo>& handle_infos
	) const {
		return make_shared<AlternatingSmoothedARAPMeshWarper<Scalar,2> >(
			points,
			triangles,
			this->collect_point_handle_verts(handle_infos),
			m_smoothing_weight
		);
	}

private:
	Scalar m_smoothing_weight;
};

END_OPTIMTOOLS_NAMESPACE
