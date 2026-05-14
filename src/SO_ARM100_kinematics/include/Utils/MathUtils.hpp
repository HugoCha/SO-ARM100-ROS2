#pragma once

#include "Global.hpp"

#include <cmath>

namespace SOArm100::Kinematics
{

template< typename T >
[[nodiscard]] T FindClosest( const T& target, const T& a, const T& b ){
	return ( std::abs( a - target ) < std::abs( b - target ) ) ? a : b;
}

[[nodiscard]] Vec3d ProjectPointOnPlane(
	const Vec3d& point,
	const Vec3d& plane_point,
	const Vec3d& plane_normal );

[[nodiscard]] Vec3d ProjectVectorOnPlane(
	const Vec3d& vector,
	const Vec3d& plane_normal
	);

[[nodiscard]] Vec3d ProjectPointOnAxis(
	const Vec3d& point,
	const Vec3d& origin,
	const Vec3d& axis );

[[nodiscard]] double Angle( const Vec3d& V1, const Vec3d& V2 );
[[nodiscard]] double SignedAngle( const Vec3d& V1, const Vec3d& V2, const Vec3d& normal );
[[nodiscard]] double AngleAroundAxis( const Vec3d& V1, const Vec3d& V2, const Vec3d& axis );

[[nodiscard]] double OrthogonalityError( const Mat3d& Q );
}