#pragma once

#include "Global.hpp"

namespace SOArm100::Kinematics
{
class Link
{
public:
Link( const Mat4d& joint_origin ) :
	joint_origin_( joint_origin ),
	length_( joint_origin_.norm() )
{
}

Link( const Link& link ) :
	joint_origin_( link.joint_origin_ ),
	length_( link.length_ )
{
}

[[nodiscard]] const Mat4d GetJointOrigin() const {
	return joint_origin_;
}

[[nodiscard]] const double GetLength() const {
	return length_;
}

private:
const Mat4d joint_origin_;
const double length_;
};
}