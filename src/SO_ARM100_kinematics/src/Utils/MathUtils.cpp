#include "Utils/MathUtils.hpp"

#include "Global.hpp"

namespace SOArm100::Kinematics
{

// ------------------------------------------------------------

Vec3d ProjectOnPlane( 
	const Vec3d& point, 
	const Vec3d& plane_point, 
	const Vec3d& plane_normal )
{
    Vec3d dir = point - plane_point;
    Vec3d project = dir - dir.dot( plane_normal ) * plane_normal;
    
    if ( project.norm() < epsilon )
        return Vec3d::Zero();

    return project;
}

// ------------------------------------------------------------

Vec3d ProjectOnAxis( 
	const Vec3d& point, 
	const Vec3d& origin, 
	const Vec3d& axis )
{
    Vec3d dir = point - origin;
    Vec3d project = dir.dot( axis ) * axis;
    
    if ( project.norm() < epsilon )
        return Vec3d::Zero();

    return project;
}

// ------------------------------------------------------------

}