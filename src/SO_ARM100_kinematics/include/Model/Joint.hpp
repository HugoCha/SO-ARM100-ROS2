#pragma once

#include "Global.hpp"

#include "JointType.hpp"
#include "Limits.hpp"
#include "Link.hpp"
#include "Pose.hpp"
#include "Twist.hpp"

#include <limits>
#include <memory>

namespace SOArm100::Kinematics::Model
{
class Joint
{
public:
Joint( const Joint& joint ) :
	Joint(
		Twist( joint.GetTwist() ),
		Link( joint.GetLink() ),
		Limits( joint.GetLimits() ) )
{
}

Joint( const Twist& twist, const Link& link, const Limits& limits ) :
	twist_( std::make_unique< const Twist >( twist ) ),
	link_( std::make_unique< const Link >( link ) ),
	limits_( std::make_unique< const Limits >( limits ) ),
	type_( GetJointType( twist ) )
{
}

Joint( const Link& link ) :
	Joint(
		Twist( Vec3d::Zero(), Vec3d::Zero() ),
		link,
		Limits( -std::numeric_limits< double >::infinity(), std::numeric_limits< double >::infinity() ) )
{
}

Joint( Joint&& ) = default;
Joint& operator = ( Joint&& ) = default;

JointType GetType() const {
	return type_;
}

const Twist& GetTwist() const {
	return *twist_;
}

const Link& GetLink() const {
	return *link_;
}

const Limits& GetLimits() const {
	return *limits_;
}

const Mat4d& OriginTransform() const {
	return link_->GetJointOriginTransform();
}

const Vec3d& Origin() const {
	return link_->GetJointOrigin();
}

const Vec3d& Axis() const {
	return twist_->GetAxis();
}

const Vec3d TransformAxis( const Mat4d& transform ) const {
	return twist_->TransformAxis( transform );
}

const struct Pose Pose( const Mat4d& transform ) const {
	struct Pose pose;
	Vec4d origin_homogenous = link_->GetJointOrigin().block< 4, 1 >( 0, 3 );
	pose.axis = twist_->TransformAxis( transform );
	pose.origin = ( transform * origin_homogenous ).head( 3 );
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
JointType type_;

std::unique_ptr< const Twist > twist_;
std::unique_ptr< const Link > link_;
std::unique_ptr< const Limits > limits_;

inline static JointType GetJointType( const Twist& twist ){
	if ( twist.IsPrismatic() )
		return JointType::PRISMATIC;
	if ( twist.IsRevolute() )
		return JointType::REVOLUTE;
	return JointType::FIXED;
}
};

using JointConstPtr = std::shared_ptr< const Joint >;

}