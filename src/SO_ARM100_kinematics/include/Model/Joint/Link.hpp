#pragma once

#include "Global.hpp"

#include "Utils/KinematicsUtils.hpp"

namespace SOArm100::Kinematics::Model
{
class Link
{
public:
Link( const std::string& name, const Mat4d& origin, double length ) :
	name_( name ),
	joint_origin_transform_( origin ),
	joint_origin_position_( Translation( origin ) ),
	length_( length )
{
}

Link() :
	Link( "", Mat4d::Identity(), 0 )
{
}

Link( const std::string& name, const Mat4d& origin, const Mat4d& child_origin ) :
	Link( name, origin, ( Translation( child_origin ) - Translation( origin ) ).norm() )
{
}

Link( const Link& link ) :
	Link( link.GetName(), link.GetJointOriginTransform(), link.GetLength() )
{
}

const std::string& GetName() const {
	return name_;
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
std::string name_;
const Mat4d joint_origin_transform_;
const Vec3d joint_origin_position_;
const double length_;
};
}