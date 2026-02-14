#pragma once

#include "Global.hpp"

#include "JointType.hpp"
#include "Limits.hpp"
#include "Link.hpp"
#include "Twist.hpp"

#include <limits>
#include <memory>

namespace SOArm100::Kinematics
{
class Joint
{
public:
Joint( const Joint& joint ) :
	Joint( Twist( joint.GetTwist() ), Link( joint.GetLink() ), Limits( joint.GetLimits() ) )
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