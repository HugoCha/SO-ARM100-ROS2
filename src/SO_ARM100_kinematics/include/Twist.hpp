#pragma once

#include "Global.hpp"

#include <memory>

namespace SOArm100::Kinematics
{
struct Limits
{
    double min;
    double max;

    inline bool SatisfyLimits( double theta ) const {
        return theta >= min && theta <= max;
    }
};

class Twist
{
public:
Twist( const Vec3d& axis, const Vec3d& point_on_axis, double min, double max );
Twist( const Twist& other ) = default;
Twist() = delete;
~Twist() = default;

[[nodiscard]] inline const Vec3d GetAxis() const {
    return twist_.block< 3, 1 >( 0, 0 );
}

[[nodiscard]] inline const Vec3d GetLinear() const {
    return twist_.block< 3, 1 >( 3, 0 );
}

[[nodiscard]] inline const Limits Limits() const {
    return limits_;
}

[[nodiscard]] inline bool IsRevolute() const {
    return GetAxis().norm() > epsilon;
}

[[nodiscard]] inline bool IsPrismatic() const {
    return GetAxis().norm() < epsilon && GetLinear().norm() > epsilon;
}

[[nodiscard]] const Mat4d ExponentialMatrix( double thetha ) const;

private:
struct Cache
{
    Mat3d omega_hat;
    Mat3d omega_hat_squared;
    Mat3d omega_cross_v;
    Mat3d omega_omegaT_v;
};

const struct Limits limits_;

const Vec6d twist_;
const Cache cache_;

static Vec6d ComputeTwist( const Vec3d& axis, const Vec3d& point_on_axis );
static Cache ComputeCache( const Vec6d& twist );
};

using TwistConstPtr = std::shared_ptr< const Twist >;

}
