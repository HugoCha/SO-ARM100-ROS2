#pragma once

#include "Types.hpp"

namespace SOArm100::Kinematics
{
class Twist
{
public:
Twist( Vec3d axis, Vec3d point_on_axis );
Twist( Vec3d axis, Mat4d transform );
Twist( const Twist& other );
~Twist();

operator Vec6d () const;

Vec3d GetAxis() const;
Vec3d GetLinear() const;

private:
Vec3d axis_;
Vec3d linear_;
};
}
