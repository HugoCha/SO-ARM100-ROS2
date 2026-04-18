#pragma once

#include "Global.hpp"

#include <memory>

namespace SOArm100::Kinematics::Model
{
class Bone
{
public:
Bone( const Vec3d& origin, const Vec3d& bone ) :
	origin_( origin ),
	direction_( bone ),
	length_( bone.norm() )
{
}

const Vec3d& Origin() const {
	return origin_;
}

const Vec3d& Direction() const {
	return direction_;
}

double Length() const {
	return length_;
}

private:
Vec3d origin_;
Vec3d direction_;
double length_;
};

using BoneConstPtr = std::shared_ptr< const Bone >;

}