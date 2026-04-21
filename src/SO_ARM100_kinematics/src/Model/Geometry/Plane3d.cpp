#include "Model/Geometry/Plane3d.hpp"

#include "Global.hpp"

namespace SOArm100::Kinematics::Model
{

// ------------------------------------------------------------

Plane3d Translate( const Plane3d& plane, const Vec3d& translation )
{
	return Plane3d(
		plane.point + translation,
		plane.normal );
}

// ------------------------------------------------------------

Plane3d Rotate( const Plane3d& plane, const Quaternion& rotation )
{
	return Plane3d(
		plane.point,
		rotation * plane.normal );
}

// ------------------------------------------------------------

Plane3d Transform( const Plane3d& plane, const Vec3d& translation, const Quaternion& rotation )
{
	return Plane3d(
		plane.point + translation,
		rotation * plane.normal );
}

// ------------------------------------------------------------

Vec3d ProjectPoint( const Plane3d& plane, const Vec3d& point )
{
	Vec3d dir = point - plane.point;
	Vec3d perp = plane.normal.dot( dir ) * plane.normal;
	return dir - perp;
}

// ------------------------------------------------------------

PointPlanePosition IsPointOnPlane( const Plane3d& plane, const Vec3d& point )
{
	Vec3d dir = point - plane.point;

	auto dot = plane.normal.dot( dir );
	if ( std::abs( dot ) < epsilon )
		return PointPlanePosition::On;

	if ( dot < epsilon )
		return PointPlanePosition::Below;

	return PointPlanePosition::Above;
}

// ------------------------------------------------------------

}