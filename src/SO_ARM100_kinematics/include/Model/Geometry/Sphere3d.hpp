#pragma once

#include "Global.hpp"

namespace SOArm100::Kinematics::Model
{
struct Sphere3d
{
	Vec3d center;
	double radius;
};

Sphere3d Translate( const Sphere3d& sphere, const Vec3d& translation );
Sphere3d Rotate( const Sphere3d& sphere, const Quaternion& rotation );
Sphere3d Transform( const Sphere3d& sphere, const Vec3d& translation, const Quaternion& rotation );
}