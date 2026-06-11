#pragma once

#include "Global.hpp"

#include "Utils/KinematicsUtils.hpp"

#include <memory>

namespace SOArm100::Kinematics::Model
{
class Link
{
public:
Link( const std::string& name, 
	  const Mat4d& home_tf, 
	  const Mat4d& local_tf, 
	  double length ) :
	name_( name ),
	home_tf_( home_tf ),
	local_tf_( local_tf ),
	length_( length )
{
}

Link( const std::string& name, 
	  const Mat4d& home_tf, 
	  const Mat4d& local_tf,
	  const Mat4d& child_tf ) :
	Link( name, home_tf, local_tf, ( Translation( child_tf ) - Translation( home_tf ) ).norm() )
{
}

Link() :
	Link( "", Mat4d::Identity(), Mat4d::Identity(), 0 )
{
}

Link( const Link& link ) :
	Link( link.GetName(), link.HomeTransform(), link.ParentJointTransform(), link.Length() )
{
}

const std::string& GetName() const {
	return name_;
}

[[nodiscard]] const Mat4d& HomeTransform() const {
	return home_tf_;
}

[[nodiscard]] const Mat4d& ParentJointTransform() const {
	return local_tf_;
}

[[nodiscard]] const double& Length() const {
	return length_;
}

private:
const std::string name_;
const Mat4d home_tf_;
const Mat4d local_tf_;
const double length_;
};

using LinkConstPtr = std::shared_ptr< const Link >;
using LinkPtr = std::shared_ptr< Link >;
}