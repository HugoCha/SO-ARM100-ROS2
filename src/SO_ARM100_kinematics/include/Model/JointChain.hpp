#pragma once

#include "Global.hpp"
#include "Joint.hpp"

#include <cstddef>
#include <memory>
#include <span>
#include <vector>

namespace SOArm100::Kinematics::Model
{
struct JointState;
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

int GetJointIndex( const Joint* joint ) const;
const Joint* GetNextJoint( const Joint* joint ) const;
const Joint* GetPreviousJoint( const Joint* joint ) const;

[[nodiscard]] std::vector< double > ActiveJointCenters() const noexcept {
	return active_joint_centers_;
}

[[nodiscard]] bool WithinLimits( const VecXd& joints ) const {
	if ( joints.size() > GetActiveJointCount() )
		return false;

	return WithinLimits( joints.data(), joints.size() );
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

bool ComputeFK(
	const std::span< const double >& thetas,
	const Mat4d& home_configuration,
	Mat4d& fk ) const {
	return ComputeFK( thetas.data(), thetas.size(), home_configuration, fk );
}

bool ComputeFK(
	const VecXd& thetas,
	const Mat4d& home_configuration,
	Mat4d& fk ) const {
	return ComputeFK( thetas.data(), thetas.size(), home_configuration, fk );
}

bool ComputeJointStatesFK(
	const std::span< const double >& thetas,
	const Mat4d& home_configuration,
	std::vector< JointState >& joint_states,
	Mat4d& fk ) const {
	return ComputeJointStatesFK( thetas.data(), thetas.size(), home_configuration, joint_states, fk );
}

bool ComputeJointStatesFK(
	const VecXd& thetas,
	const Mat4d& home_configuration,
	std::vector< JointState >& joint_states,
	Mat4d& fk ) const {
	return ComputeJointStatesFK( thetas.data(), thetas.size(), home_configuration, joint_states, fk );
}

bool ComputeJointPosesFK(
	const std::span< const double >& thetas,
	const Mat4d& home_configuration,
	std::vector< Mat4d >& joints_fk,
	Mat4d& fk ) const
{
	return ComputeJointPosesFK( thetas.data(), thetas.size(), home_configuration, joints_fk, fk );
}

bool ComputeJointPosesFK(
	const VecXd& thetas,
	const Mat4d& home_configuration,
	std::vector< Mat4d >& joints_fk,
	Mat4d& fk ) const
{
	return ComputeJointPosesFK( thetas.data(), thetas.size(), home_configuration, joints_fk, fk );
}

[[nodiscard]] VecXd RandomValidJoints(
	random_numbers::RandomNumberGenerator& rng,
	double margin_percent = 0.05 ) const noexcept;

[[nodiscard]] VecXd RandomValidJointsNear(
	random_numbers::RandomNumberGenerator& rng,
	const VecXd& joints,
	double distance = 0.1,
	double margin_percent = 0.05 ) const noexcept;

[[nodiscard]] VecXd RandomValidJointsNearWrapped(
	random_numbers::RandomNumberGenerator& rng,
	const VecXd& joints,
	double min_limit_span,
	double distance = 0.1,
	double margin_percent = 0.05 ) const noexcept;

[[nodiscard]] VecXd RandomValidJointsNearCentered(
	random_numbers::RandomNumberGenerator& rng,
	const VecXd& joints,
	double distance = 0.1,
	double margin_percent = 0.05 ) const noexcept;

[[nodiscard]] JointChain SubChain( JointConstPtr start, JointConstPtr end ) const;

private:
std::vector< JointConstPtr > joints_;
std::vector< JointConstPtr > active_joints_;
std::vector< double > active_joint_centers_;

JointChain( const std::span< JointConstPtr const >& joints );

bool ComputeFK(
	const double* thetas,
	int n_joints,
	const Mat4d& home_configuration,
	Mat4d& fk ) const noexcept;

bool ComputeJointStatesFK(
	const double* thetas,
	int n_joints,
	const Mat4d& home_configuration,
	std::vector< JointState >& joint_states,
	Mat4d& fk ) const noexcept;

bool ComputeJointPosesFK(
	const double* thetas,
	int n_joints,
	const Mat4d& home_configuration,
	std::vector< Mat4d >& joints_fk,
	Mat4d& fk ) const noexcept;

bool WithinLimits( const double* joints, int n_joints ) const;

void Add( JointConstPtr joint );
};

using JointChainConstPtr = std::shared_ptr< const JointChain >;
}