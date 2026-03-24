#pragma once

#include "Global.hpp"

#include <cmath>

namespace SOArm100::Kinematics
{

template< typename T >
T FindClosest( const T& target, const T& a, const T& b ){
	return ( std::abs( a - target ) < std::abs( b - target ) ) ? a : b;
}

Vec3d ProjectOnPlane( 
	const Vec3d& point, 
	const Vec3d& plane_point, 
	const Vec3d& plane_normal );

Vec3d ProjectOnAxis(
	const Vec3d& point,
	const Vec3d& origin,
	const Vec3d& axis );
}