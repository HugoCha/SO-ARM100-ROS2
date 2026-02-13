#include "Twist.hpp"

#include "KinematicsUtils.hpp"

#include <Eigen/Dense>

namespace SOArm100::Kinematics
{
// ------------------------------------------------------------

Twist::Twist( const Vec3d& axis, const Vec3d& point_on_axis, double min, double max ) :
	twist_( ComputeTwist( axis, point_on_axis ) ),
	cache_( ComputeCache( twist_ ) ),
	limits_( min, max )
{
}

// ------------------------------------------------------------

Vec6d Twist::ComputeTwist( const Vec3d& axis, const Vec3d& point_on_axis )
{
	Vec6d twist;
	const auto& axis_normalized = axis.normalized();
	const auto& linear = -axis_normalized.cross( point_on_axis );

	twist << axis_normalized, linear;

	return twist;
}

// ------------------------------------------------------------

Twist::Cache Twist::ComputeCache( const Vec6d& twist )
{
	Cache cache;
	const Vec3d omega = twist.block< 3, 1 >( 0, 0 );
	const Vec3d v = twist.block< 3, 1 >( 3, 0 );

	cache.omega_hat = SkewMatrix( omega );
	cache.omega_hat_squared = cache.omega_hat * cache.omega_hat;
	cache.omega_cross_v = cache.omega_hat * v;
	cache.omega_omegaT_v = omega * omega.transpose() * v;

	return cache;
}

// ------------------------------------------------------------

const Mat4d Twist::ExponentialMatrix( double thetha ) const
{
	Mat4d exponential = Mat4d::Identity();
	Mat3d R;
	Vec3d t;

	R.noalias() = cache_.omega_hat * sin( thetha ) + cache_.omega_hat_squared * ( 1 - cos( thetha ) );
	t.noalias() = ( -R ) * ( cache_.omega_cross_v ) + cache_.omega_omegaT_v * thetha;

	exponential.block< 3, 3 >( 0, 0 ).noalias() += R;
	exponential.block< 3, 1 >( 0, 1 ).noalias() = t;

	return exponential;
}

// ------------------------------------------------------------

}
