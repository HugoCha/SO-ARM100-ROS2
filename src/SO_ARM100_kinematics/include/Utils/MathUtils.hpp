#pragma once

#include "Global.hpp"

#include <cmath>

namespace SOArm100::Kinematics
{

template< typename T >
T FindClosest( const T& target, const T& a, const T& b ){
	return ( std::abs( a - target ) < std::abs( b - target ) ) ? a : b;
}

Vec3d ProjectPointOnPlane(
	const Vec3d& point,
	const Vec3d& plane_point,
	const Vec3d& plane_normal );

Vec3d ProjectVectorOnPlane(
	const Vec3d& vector,
	const Vec3d& plane_normal
	);

Vec3d ProjectPointOnAxis(
	const Vec3d& point,
	const Vec3d& origin,
	const Vec3d& axis );

double Angle( const Vec3d& V1, const Vec3d& V2 );
double SignedAngle( const Vec3d& V1, const Vec3d& V2, const Vec3d& normal );
}