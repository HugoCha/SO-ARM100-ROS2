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

double Angle( const Vec3d& V1, const Vec3d& V2 )
{
	Vec3d V1xV2 = V1.cross( V2 );
	double V1dotV2 = V1.dot( V2 );
	
	double angle = 0;

	if ( V1xV2.norm() < epsilon )
	{
		if ( V1dotV2 > 0 ) return 0.0;
		return M_PI;
	}

    double sin_angle = V1xV2.norm();
    double cos_angle = V1dotV2;
    return std::atan2( sin_angle, cos_angle );
}

// ------------------------------------------------------------

double SignedAngle( const Vec3d& V1, const Vec3d& V2, const Vec3d& normal )
{
	Vec3d V1xV2 = V1.cross( V2 );
	double V1dotV2 = V1.dot( V2 );
	
    double sin_angle = V1xV2.norm();
    double cos_angle = V1dotV2;
    double angle  = std::atan2( sin_angle, cos_angle );

    return ( normal.dot( V1xV2 ) < 0 ) ? -angle : angle;
}

// ------------------------------------------------------------

}