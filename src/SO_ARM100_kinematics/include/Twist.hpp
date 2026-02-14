#pragma once

#include "Global.hpp"

#include <memory>

namespace SOArm100::Kinematics
{
class Twist
{
public:
Twist();
Twist( const Vec3d& linear );
Twist( const Vec3d& axis, const Vec3d& point_on_axis );
Twist( const Twist& other ) = default;
~Twist() = default;

Twist operator = ( const Twist& other ){
	return Twist( other.twist_ );
}

[[nodiscard]] inline operator const Vec6d () const {
	return twist_;
}

[[nodiscard]] inline const Vec3d GetAxis() const {
	return axis_;
}

[[nodiscard]] inline const Vec3d GetLinear() const {
	return linear_;
}

[[nodiscard]] inline bool IsRevolute() const {
	return axis_.norm() > epsilon;
}

[[nodiscard]] inline bool IsPrismatic() const {
	return axis_.norm() < epsilon && linear_.norm() > epsilon;
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

Twist( const Vec6d& twist );

const Vec6d twist_;
const Vec3d axis_;
const Vec3d linear_;

const Cache cache_;

static Vec6d ComputeTwist( const Vec3d& linear );
static Vec6d ComputeTwist( const Vec3d& axis, const Vec3d& point_on_axis );
static Cache ComputeCache( const Vec6d& twist );
};

using TwistConstPtr = std::shared_ptr< const Twist >;

}
