#pragma once

#include "Global.hpp"

namespace SOArm100::Kinematics::Model
{
struct Plane3d
{
Vec3d point;
Vec3d normal;
};

enum class PointPlanePosition
{
On,
Above,
Below
};

Plane3d Translate( const Plane3d& plane, const Vec3d& translation );
Plane3d Rotate( const Plane3d& plane, const Quaternion& rotation );
Plane3d Transform( const Plane3d& plane, const Vec3d& translation, const Quaternion& rotation );

Vec3d ProjectPoint( const Plane3d& plane, const Vec3d& point );
PointPlanePosition IsPointOnPlane( const Plane3d& plane, const Vec3d& point );
}