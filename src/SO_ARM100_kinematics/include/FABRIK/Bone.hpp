#pragma once

#include "Global.hpp"

namespace SOArm100::Kinematics::Model
{
class Bone
{
public:
Bone( const Vec3d& bone ) :
	direction_( bone ),
	length_( bone.norm() )
{
}

const Vec3d& Direction() const {
	return direction_;
}

double Length() const {
	return length_;
}

private:
Vec3d direction_;
double length_;
};
}