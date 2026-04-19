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

}