#pragma once

#include "Global.hpp"

#include <memory>
#include <optional>

namespace SOArm100::Kinematics
{
class Twist
{
public:
Twist();
Twist( const Vec3d& linear );
Twist( const Vec3d& axis, const Vec3d& point_on_axis );
Twist( const Twist& other ) = default;
Twist( Twist&& ) = default;

~Twist() = default;

Twist& operator = ( Twist&& ) = default;

[[nodiscard]] inline operator const Vec6d () const {
	return twist_;
}

[[nodiscard]] inline const Vec3d GetAxis() const {
	return axis_;
}

[[nodiscard]] inline const Vec3d GetLinear() const {
	return linear_;
}

[[nodiscard]] const Vec3d TransformAxis( const Mat4d& transform ) const;

[[nodiscard]] inline bool IsRevolute() const {
	return IsRevolute( twist_ );
}

[[nodiscard]] inline bool IsPrismatic() const {
	return IsPrismatic( twist_ );
}

[[nodiscard]] const Mat4d ExponentialMatrix( double thetha ) const;

private:
struct Cache
{
	Mat3d omega_hat;
	Mat3d omega_hat_squared;
	Vec3d omega_cross_v;
	Vec3d omega_omegaT_v;
};

Vec6d twist_;
Vec3d axis_;
Vec3d linear_;

std::optional< Cache > cache_;

static bool IsRevolute( const Vec6d& twist ){
	return twist.head( 3 ).norm() > epsilon;
}

static bool IsPrismatic( const Vec6d& twist ){
	return twist.head( 3 ).norm() < epsilon && twist.tail( 3 ).norm() > epsilon;
}

static Vec6d ComputeTwist( const Vec3d& linear );
static Vec6d ComputeTwist( const Vec3d& axis, const Vec3d& point_on_axis );
static std::optional< Cache > ComputeCache( const Vec6d& twist );
};

using TwistConstPtr = std::shared_ptr< const Twist >;

}
