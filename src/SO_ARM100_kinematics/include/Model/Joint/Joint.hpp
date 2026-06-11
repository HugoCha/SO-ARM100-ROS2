#pragma once

#include "Global.hpp"

#include "JointType.hpp"
#include "Limits.hpp"
#include "Model/Geometry/Pose.hpp"
#include "Model/Joint/Link.hpp"
#include "Twist.hpp"
#include "Utils/KinematicsUtils.hpp"

#include <memory>

namespace SOArm100::Kinematics::Model
{
class Joint
{
public:
Joint( const Joint& joint ) :
	Joint(
		joint.GetName(),
		joint.OriginTransform(),
		Twist( joint.GetTwist() ),
		Limits( joint.GetLimits() ),
		joint.parent_link_,
		joint.child_link_ )
{
}

Joint( const std::string& name,
	   const Mat4d& joint_home_tf,
	   const Twist& twist,
	   const Limits& limits,
	   LinkConstPtr parent_link,
	   LinkConstPtr child_link ) :
	name_( name ),
	home_global_tf_( joint_home_tf ),
	home_global_pos_( Translation( joint_home_tf ) ),
	twist_( std::make_unique< const Twist >( twist ) ),
	limits_( std::make_unique< const Limits >( limits ) ),
	type_( GetJointType( twist ) ),
	parent_link_( parent_link ),
	child_link_( child_link )
{
}

Joint( Joint&& ) = default;
Joint& operator = ( Joint&& ) = default;

const JointType& GetType() const {
	return type_;
}

const std::string& GetName() const {
	return name_;
}

const Link* const GetParentLink() const {
	return parent_link_.get();
}

const Link* const GetChildLink() const {
	return child_link_.get();
}

void SetChildLink( LinkConstPtr child_link ) {
	child_link_ = child_link;
}

const Twist& GetTwist() const {
	return *twist_;
}

const Limits& GetLimits() const {
	return *limits_;
}

const Mat4d& OriginTransform() const {
	return home_global_tf_;
}

const Vec3d& Origin() const {
	return home_global_pos_;
}

const Vec3d& Axis() const {
	return twist_->IsRevolute() ? twist_->Omega() : twist_->V();
}

const Vec3d TransformAxis( const Mat4d& transform ) const {
	return Rotation( transform ) * Axis();
}

const struct Pose Pose( const Mat4d& transform ) const {
	struct Pose pose;
	pose.axis = Rotation( transform ) * Axis();
	pose.origin = ( transform * Origin().homogeneous() ).head( 3 );
	return pose;
}

bool IsRevolute() const {
	return type_ == JointType::REVOLUTE;
}

bool IsPrismatic() const {
	return type_ == JointType::PRISMATIC;
}

bool IsFixed() const {
	return type_ == JointType::FIXED;
}

private:
std::string name_;
JointType type_;
Vec3d home_global_pos_;
Mat4d home_global_tf_;

LinkConstPtr parent_link_;
LinkConstPtr child_link_;

TwistConstPtr twist_;
LimitsConstPtr limits_;

inline static JointType GetJointType( const Twist& twist ){
	if ( twist.IsPrismatic() )
		return JointType::PRISMATIC;
	if ( twist.IsRevolute() )
		return JointType::REVOLUTE;
	return JointType::FIXED;
}
};

using JointConstPtr = std::shared_ptr< const Joint >;
using JointPtr = std::shared_ptr< Joint >;

}