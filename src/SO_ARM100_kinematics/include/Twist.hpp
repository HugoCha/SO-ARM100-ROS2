#pragma once

#include "Types.hpp"

namespace SOArm100::Kinematics
{
class Twist
{
public:
Twist( const Vec3d& axis, const Vec3d& point_on_axis );
Twist( const Vec3d& axis, const Mat4d& transform );
Twist( const Twist& other ) = default;
~Twist() = default;

operator Vec6d () const;

[[nodiscard]] Vec3d GetAxis() const;
[[nodiscard]] Vec3d GetLinear() const;

private:
Vec3d axis_;
Vec3d linear_;
};
}
