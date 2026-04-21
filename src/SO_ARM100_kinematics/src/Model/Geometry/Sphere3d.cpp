#include "Model/Geometry/Sphere3d.hpp"

#include "Global.hpp"
#include <cstdlib>

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
		sphere.center + translation,
		sphere.radius );
}

// ------------------------------------------------------------

Vec3d ProjectPoint( const Sphere3d& sphere, const Vec3d& point )
{
    Vec3d dir = point - sphere.center;
    return sphere.center + sphere.radius * dir.normalized();
}

// ------------------------------------------------------------

PointSpherePosition IsPointOnSphere( const Sphere3d& sphere, const Vec3d& point )
{
    Vec3d dir = point - sphere.center;
    if ( std::abs( dir.norm() - sphere.radius ) < epsilon )
        return PointSpherePosition::On;

    if ( dir.norm() > sphere.radius )
        return PointSpherePosition::Outside;

    return PointSpherePosition::Inside;
}

// ------------------------------------------------------------

}