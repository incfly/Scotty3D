
#include "transform.h"

Mat4 Transform::local_to_parent() const {
	return Mat4::translate(translation) * rotation.to_mat() * Mat4::scale(scale);
}

Mat4 Transform::parent_to_local() const {
	return Mat4::scale(1.0f / scale) * rotation.inverse().to_mat() * Mat4::translate(-translation);
}

Mat4 Transform::local_to_world() const {
	auto par = parent.lock();
	if (!par) {
		return local_to_parent();
	}
	return par->local_to_world() * local_to_parent();
}

// Single pass! Once understand the transformation, mat4, math, things fit like precise machine!
Mat4 Transform::world_to_local() const {
	auto neg_parent_mat = Mat4::I;
	auto par = parent.lock();
	if (par) {
		neg_parent_mat = par->world_to_local();
	}
	auto neg_trans = -this->translation;
	
	// Because of using quarternion, we can just negate, very simple and elegant!
	auto neg_rotate = -this->rotation;

	auto neg_scale = Vec3{1/this->scale.x, 1/this->scale.y, 1/this->scale.z};

	auto neg_all_mat = Mat4::scale(neg_scale) * neg_rotate.to_mat() * Mat4::translate(neg_trans);
	return neg_all_mat * neg_parent_mat;
}

bool operator!=(const Transform& a, const Transform& b) {
	return a.parent.lock() != b.parent.lock() || a.translation != b.translation ||
	       a.rotation != b.rotation || a.scale != b.scale;
}
