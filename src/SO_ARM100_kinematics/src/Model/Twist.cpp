#include "Model/Twist.hpp"

#include "Utils/KinematicsUtils.hpp"

#include <Eigen/Dense>
#include <optional>

namespace SOArm100::Kinematics
{

// ------------------------------------------------------------

Twist::Twist() :
	Twist( Vec3d::Zero() )
{
}

// ------------------------------------------------------------

Twist::Twist( const Vec3d& linear ) :
	twist_( ComputeTwist( linear ) ),
	axis_( twist_.head( 3 ) ),
	linear_( twist_.tail( 3 ) ),
	cache_( ComputeCache( twist_ ) )
{
}

// ------------------------------------------------------------

Twist::Twist( const Vec3d& axis, const Vec3d& point_on_axis ) :
	twist_( ComputeTwist( axis, point_on_axis ) ),
	axis_( twist_.head( 3 ) ),
	linear_( twist_.tail( 3 ) ),
	cache_( ComputeCache( twist_ ) )
{
}

// ------------------------------------------------------------

Vec6d Twist::ComputeTwist( const Vec3d& linear )
{
	Vec6d twist;
	twist.head( 3 ) = Vec3d::Zero();
	twist.tail( 3 ) = linear.normalized();
	return twist;
}

// ------------------------------------------------------------

Vec6d Twist::ComputeTwist( const Vec3d& axis, const Vec3d& point_on_axis )
{
	Vec6d twist;
	const Vec3d& axis_normalized = axis.normalized();
	const Vec3d& linear = -axis_normalized.cross( point_on_axis );
	twist.head( 3 ) = axis_normalized;
	twist.tail( 3 ) = linear;
	return twist;
}

// ------------------------------------------------------------

std::optional< Twist::Cache > Twist::ComputeCache( const Vec6d& twist )
{
	if ( !IsRevolute( twist ) )
		return std::nullopt;

	const Vec3d omega = twist.head( 3 );
	const Vec3d v = twist.tail( 3 );

	Cache cache;
	cache.omega_hat = SkewMatrix( omega );
	cache.omega_hat_squared = cache.omega_hat * cache.omega_hat;
	cache.omega_cross_v = cache.omega_hat * v;
	cache.omega_omegaT_v = omega * omega.transpose() * v;

	return cache;
}

// ------------------------------------------------------------

const Vec3d Twist::TransformAxis( const Mat4d& transform ) const
{
	return Rotation( transform ) * axis_;
}

// ------------------------------------------------------------

const Mat4d Twist::ExponentialMatrix( double thetha ) const
{
	Mat4d exponential = Mat4d::Identity();
	Mat3d R;
	Vec3d t;

	if ( !cache_ )
	{
		exponential.block< 3, 1 >( 0, 3 ).noalias() = linear_ * thetha;
	}
	else
	{
		R.noalias() = cache_->omega_hat * sin( thetha ) + cache_->omega_hat_squared * ( 1 - cos( thetha ) );
		t.noalias() = ( -R * cache_->omega_cross_v ) + cache_->omega_omegaT_v * thetha;
		exponential.block< 3, 3 >( 0, 0 ).noalias() += R;
		exponential.block< 3, 1 >( 0, 3 ).noalias() = t;
	}

	return exponential;
}

// ------------------------------------------------------------

}
