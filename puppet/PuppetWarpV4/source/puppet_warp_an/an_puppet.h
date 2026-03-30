#pragma once

/** \file
 * Defines the puppet type used within Adobe Animate.
 */

#include "puppet_warp_an/common.h"
#include "puppet_warp_an/an_warper.h"

BEGIN_OPTIMTOOLS_NAMESPACE

/**
 * Creates a puppet backed by full ARAP-like mesh deformation, but with some
 * extra bells and whistles.  Intended for use in Adobe Animate.  Extra festures
 * include:
 *
 * * support for segment-handles
 */
template<typename Scalar>
class AnPuppet : public FullMeshWarperPuppet<Scalar, AnPuppetWarper<Scalar> > {
	typedef FullMeshWarperPuppet<Scalar, AnPuppetWarper<Scalar> > Super;
	USING_OPTIMTOOLS_MESH_TYPES(typename,Scalar,2);
	
public:
	using typename Super::Figure;
	
protected:
	using typename Super::HandleInfo;

public:
	AnPuppet() = default;

	AnPuppet(const Figure& figure) :
		Super(figure)
	{ }

	AnPuppet(const Figure& figure, const Scalar& smoothing_weight) :
		Super(figure),
		m_smoothing_weight(smoothing_weight)
	{ }

	void reset_solver() {
		this->warper()->reset_solver();
	}

protected:
	virtual shared_ptr<AnPuppetWarper<Scalar> > init_warper_from_mesh(
		const PointVector& points,
		const TriangleVector& triangles,
		const std::vector<HandleInfo>& handle_infos
	) const {
		return make_shared<AnPuppetWarper<Scalar> >(
			points,
			triangles,
			handle_infos,
			m_smoothing_weight
		);
	}

private:
	Scalar m_smoothing_weight{1};
};

END_OPTIMTOOLS_NAMESPACE
