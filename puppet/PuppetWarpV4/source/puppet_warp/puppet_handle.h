#pragma once

/** \file
 * A puppet is controlled by specifying parameters for each of a set of
 * *handles*.
 */

#include "optimtools/all.h"

BEGIN_OPTIMTOOLS_NAMESPACE


/**
 * A simple class for identifying a handle of a puppet.  This gives the type
 * of the handle, and an `Int` used to uniquely identify the handle.  These
 * identifiers are guaranteed to be unique within each puppet (but not between
 * puppets), be non-negative, and not to change, but do *not* have to occupy
 * consecutive integers.
 */
class PuppetHandle : public serialization::Serializable {
public:
	inline PuppetHandle() :
		m_id(-1)
	{ }

	inline PuppetHandle(Int id) :
		m_id(id)
	{ }

	inline virtual ~PuppetHandle() { }

	inline Int id() const {
		return m_id;
	}

	inline virtual void serialize(serialization::Archive& ar) override {
		serialize_impl(ar);
	}

	template<typename Archive>
	inline void serialize(Archive& ar) {
		serialize_impl(ar);
	}

protected:
	inline Int& id() {
		return m_id;
	}

	template<typename Archive>
	void serialize_impl(Archive& ar) {
		enum {
			ID = 0
		};

		ar.object([&](auto& ar) {
			ar("id",ID) & id();
		});
	}

private:
	Int m_id;
};

END_OPTIMTOOLS_NAMESPACE
