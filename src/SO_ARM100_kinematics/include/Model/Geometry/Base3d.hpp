#pragma once

#include "Global.hpp"

namespace SOArm100::Kinematics::Model
{
struct Base3d
{
	Vec3d x;
	Vec3d y;
	Vec3d z;
};

Base3d Translate( const Base3d& base, const Vec3d& translation );
Base3d Rotate( const Base3d& base, const Quaternion& rotation );
Base3d Transform( const Base3d& base, const Vec3d& translation, const Quaternion& rotation );
}