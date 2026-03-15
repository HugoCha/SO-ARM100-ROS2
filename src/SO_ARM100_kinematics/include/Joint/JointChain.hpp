#pragma once

#include "Global.hpp"
#include "Joint.hpp"

#include <cstddef>
#include <memory>
#include <span>
#include <vector>

namespace SOArm100::Kinematics
{
struct Pose;

class JointChain
{
public:
JointChain( int n );
JointChain( const JointChain& chain ) :
	joints_( chain.joints_ ),
	active_joints_( chain.active_joints_ ),
	active_joint_centers_( chain.active_joint_centers_ )
{
}

JointChain( JointChain&& ) = default;
JointChain& operator = ( JointChain&& ) = default;

[[nodiscard]] std::span< JointConstPtr const > GetJoints() const {
	return joints_;
}

[[nodiscard]] std::span< JointConstPtr const > GetActiveJoints() const {
	return active_joints_;
}

JointConstPtr GetActiveJoint( int i ) const {
	if ( i < 0 || i >= static_cast< int >( active_joints_.size() ) )
		throw std::out_of_range( "Invalid active joint index" );
	return active_joints_[i];
}

const Twist& GetActiveJointTwist( int i ) const {
	if ( i < 0 || i >= static_cast< int >( active_joints_.size() ) )
		throw std::out_of_range( "Invalid active joint index" );
	return active_joints_[i]->GetTwist();
}

const Limits& GetActiveJointLimits( int i ) const {
	if ( i < 0 || i >= static_cast< int >( active_joints_.size() ) )
		throw std::out_of_range( "Invalid active joint index" );
	return active_joints_[i]->GetLimits();
}

const Link& GetActiveJointLink( int i ) const {
	if ( i < 0 || i >= active_joints_.size() )
		throw std::out_of_range( "Invalid active joint index" );
	return active_joints_[i]->GetLink();
}

int GetJointIndex( const JointConstPtr& joint ) const;
const Joint* GetNextJoint( const JointConstPtr& joint ) const;
const Joint* GetPreviousJoint( const JointConstPtr& joint ) const;

[[nodiscard]] std::vector< double > ActiveJointCenters() const noexcept {
	return active_joint_centers_;
}

[[nodiscard]] bool WithinLimits( const VecXd& joints ) const;

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

void ComputeFK(
	const std::span< const double >& thetas,
	const Mat4d& home_configuration,
	Mat4d& fk ) const 
{
	if ( thetas.size() != GetActiveJointCount() )
		throw std::invalid_argument( "thetas size must match joint count" );
	ComputeFK( thetas.data(), home_configuration, fk );
}

void ComputeFK(
	const VecXd& thetas,
	const Mat4d& home_configuration,
	Mat4d& fk ) const 
{
	if ( thetas.size() != GetActiveJointCount() )
		throw std::invalid_argument( "thetas size must match joint count" );
	ComputeFK( thetas.data(), home_configuration, fk );
}

void ComputeIntermediateFK(
	const std::span< const double >& thetas,
	const Mat4d& home_configuration,
	std::vector< Pose >& joints_fk,
	Mat4d& fk ) const 
{
	if ( thetas.size() != GetActiveJointCount() )
		throw std::invalid_argument( "thetas size must match joint count" );
	ComputeIntermediateFK( thetas.data(), home_configuration, joints_fk, fk );
}

void ComputeIntermediateFK(
	const VecXd& thetas,
	const Mat4d& home_configuration,
	std::vector< Pose >& joints_fk,
	Mat4d& fk ) const 
{
	if ( thetas.size() != GetActiveJointCount() )
		throw std::invalid_argument( "thetas size must match joint count" );
	ComputeIntermediateFK( thetas.data(), home_configuration, joints_fk, fk );
}

void ComputeIntermediateFK(
	const std::span< const double >& thetas,
	const Mat4d& home_configuration,
	std::vector< Mat4d >& joints_fk,
	Mat4d& fk ) const 
{
	if ( thetas.size() != GetActiveJointCount() )
		throw std::invalid_argument( "thetas size must match joint count" );
	ComputeIntermediateFK( thetas.data(), home_configuration, joints_fk, fk );
}

void ComputeIntermediateFK(
	const VecXd& thetas,
	const Mat4d& home_configuration,
	std::vector< Mat4d >& joints_fk,
	Mat4d& fk ) const 
{
	if ( thetas.size() != GetActiveJointCount() )
		throw std::invalid_argument( "thetas size must match joint count" );
	ComputeIntermediateFK( thetas.data(), home_configuration, joints_fk, fk );
}

[[nodiscard]] JointChain SubChain( JointConstPtr start, JointConstPtr end ) const;

private:
std::vector< JointConstPtr > joints_;
std::vector< JointConstPtr > active_joints_;
std::vector< double > active_joint_centers_;

JointChain( const std::span< JointConstPtr const >& joints );

void ComputeFK(
	const double* thetas,
	const Mat4d& home_configuration,
	Mat4d& fk ) const noexcept;

void ComputeIntermediateFK(
	const double* thetas,
	const Mat4d& home_configuration,
	std::vector< Pose >& joints_fk,
	Mat4d& fk ) const noexcept;

void ComputeIntermediateFK(
	const double* thetas,
	const Mat4d& home_configuration,
	std::vector< Mat4d >& joints_fk,
	Mat4d& fk ) const noexcept;

void Add( JointConstPtr joint );
};
}