#pragma once

#include "Global.hpp"

namespace SOArm100::Kinematics::Model
{
struct Sphere3d
{
	Vec3d center;
	double radius;
};

enum class PointSpherePosition
{
On,
Inside,
Outside
};

Sphere3d Translate( const Sphere3d& sphere, const Vec3d& translation );
Sphere3d Rotate( const Sphere3d& sphere, const Quaternion& rotation );
Sphere3d Transform( const Sphere3d& sphere, const Vec3d& translation, const Quaternion& rotation );

Vec3d ProjectPoint( const Sphere3d& sphere, const Vec3d& point );
PointSpherePosition IsPointOnSphere( const Sphere3d& sphere, const Vec3d& point );
}