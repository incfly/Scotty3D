
#pragma once

#include <memory>

#include "introspect.h"
#include "../lib/mathlib.h"

class Transform {
public:
	Transform() = default;
	// Important: `that record the scaling, rotation, and translation (in that order!)`.
	// Vec euler is the degree of rotation around the x, y, z axes, between -1, 1. (or 180 not sure which metric)
	// Quat calculate the rotation in a quaternion, ax + by + cz + d. why quaternion can be used for descirbing rotation?
	Transform(Vec3 t, Vec3 euler, Vec3 s) : translation(t), rotation(Quat::euler(euler)), scale(s) {
	}
	Transform(Vec3 t, Quat q, Vec3 s) : translation(t), rotation(q), scale(s) {
	}

	std::weak_ptr<Transform> parent;
	Vec3 translation = Vec3{0.0f, 0.0f, 0.0f};
	Quat rotation = Quat::euler(Vec3{0.0f, 0.0f, 0.0f});
	Vec3 scale = Vec3{1.0f, 1.0f, 1.0f};

	Mat4 local_to_parent() const;
	Mat4 parent_to_local() const;
	Mat4 local_to_world() const;
	Mat4 world_to_local() const;

	template< Intent I, typename F, typename T >
	static void introspect(F&& f, T&& t) {
		if constexpr (I != Intent::Animate) f("parent", t.parent);
		f("translation", t.translation);
		f("rotation", t.rotation);
		f("scale", t.scale);
	}
	static inline const char *TYPE = "Transform";
};

bool operator!=(const Transform& a, const Transform& b);
