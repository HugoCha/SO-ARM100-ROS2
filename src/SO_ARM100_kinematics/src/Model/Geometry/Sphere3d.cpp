#include "Model/Geometry/Sphere3d.hpp"

#include "Global.hpp"

namespace SOArm100::Kinematics::Model
{

// ------------------------------------------------------------

Sphere3d Translate( const Sphere3d& sphere, const Vec3d& translation )
{
	return Sphere3d(
		sphere.center + translation,
		sphere.radius );
}

// ------------------------------------------------------------

Sphere3d Rotate( const Sphere3d& sphere, const Quaternion& rotation )
{
	return Sphere3d(
		sphere.center,
		sphere.radius );
}

// ------------------------------------------------------------

Sphere3d Transform( const Sphere3d& sphere, const Vec3d& translation, const Quaternion& rotation )
{
	return Sphere3d(
		sphere.center,
		sphere.radius );
}

// ------------------------------------------------------------

}