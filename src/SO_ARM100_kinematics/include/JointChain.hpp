#pragma once

#include "Joint.hpp"

#include <cstddef>
#include <memory>
#include <span>

namespace SOArm100::Kinematics
{
class JointChain
{
public:
JointChain( int n );
JointChain( const JointChain& chain ) :
	joints_( chain.joints_ ),
	active_joints_( chain.active_joints_ )
{
}

[[nodiscard]] std::span< JointConstPtr const > GetJoints() const {
	return joints_;
}

[[nodiscard]] std::span< JointConstPtr const > GetActiveJoints() const {
	return active_joints_;
}

JointConstPtr const GetActiveJoint( int i ) const {
	if ( i < 0 || i >= active_joints_.size() )
		throw std::out_of_range( "Invalid active joint index" );
	return active_joints_[i];
}

const Twist& GetActiveJointTwist( int i ) const {
	if ( i < 0 || i >= active_joints_.size() )
		throw std::out_of_range( "Invalid active joint index" );
	return active_joints_[i]->GetTwist();
}

const Limits& GetActiveJointLimits( int i ) const {
	if ( i < 0 || i >= active_joints_.size() )
		throw std::out_of_range( "Invalid active joint index" );
	return active_joints_[i]->GetLimits();
}

const Link& GetActiveJointLink( int i ) const {
	if ( i < 0 || i >= active_joints_.size() )
		throw std::out_of_range( "Invalid active joint index" );
	return active_joints_[i]->GetLink();
}

[[nodiscard]] bool Empty() const {
	return joints_.empty();
}

[[nodiscard]] size_t GetJointCount() const {
	return joints_.size();
}

[[nodiscard]] size_t GetActiveJointCount() const {
	return active_joints_.size();
}

void Add( const Twist& twist, const Link& link, const Limits& limits ){
	Add( std::make_shared< const Joint >( twist, link, limits ) );
}

[[nodiscard]] const JointChain SubChain( int start_index, int count ) const;

private:
std::vector< JointConstPtr > joints_;
std::vector< JointConstPtr > active_joints_;

JointChain( const std::span< JointConstPtr const >& joints );

void Add( JointConstPtr joint );
};
}