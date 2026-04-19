#pragma once

#include "Global.hpp"

#include "Utils/KinematicsUtils.hpp"

namespace SOArm100::Kinematics::Model
{
class Link
{
public:
Link( const Mat4d& origin, double length ) :
	joint_origin_transform_( origin ),
	joint_origin_position_( Translation( origin ) ),
	length_( length )
{
}

Link() :
	Link( Mat4d::Identity(), 0 )
{
}

Link( const Mat4d& origin, const Mat4d& child_origin ) :
	Link( origin, ( Translation( child_origin ) - Translation( origin ) ).norm() )
{
}

Link( const Link& link ) :
	Link( link.joint_origin_transform_, link.length_ )
{
}

[[nodiscard]] const Vec3d& GetJointOrigin() const {
	return joint_origin_position_;
}

[[nodiscard]] const Mat4d& GetJointOriginTransform() const {
	return joint_origin_transform_;
}

[[nodiscard]] const double GetLength() const {
	return length_;
}

private:
const Mat4d joint_origin_transform_;
const Vec3d joint_origin_position_;
const double length_;
};
}