#pragma once

#include "Global.hpp"
#include "Joint.hpp"
#include "Model/Joint/Link.hpp"
#include "Utils/Converter.hpp"

#include <cstddef>
#include <memory>
#include <moveit/robot_model/joint_model.hpp>
#include <span>
#include <stdexcept>
#include <vector>

namespace SOArm100::Kinematics::Model
{
struct JointState;
struct Pose;

class JointChain
{
public:
// JointChain( const JointChain& chain ) :
// 	joints_( chain.joints_ ),
// 	links_( chain.links_ ),
// 	active_joints_( chain.active_joints_ ),
// 	active_joint_centers_( chain.active_joint_centers_ ),
// 	parent_link_joint_map_( chain.parent_link_joint_map_ ),
// 	child_link_joint_map_( chain.child_link_joint_map_ ),
// 	joint_map_( chain.joint_map_ ),
// 	link_map_( chain.link_map_ ),
// 	link_names_( chain.link_names_ ),
// 	joint_names_( chain.joint_names_ )
// {
// }

JointChain( const JointChain& ) = delete;
JointChain& operator=( const JointChain& ) = delete;
JointChain( JointChain&& ) = default;
JointChain& operator = ( JointChain&& ) = default;

[[nodiscard]] std::span< JointConstPtr const > GetJoints() const {
	return joints_;
}

[[nodiscard]] std::span< LinkConstPtr const > GetLinks() const {
	return links_;
}

[[nodiscard]] std::span< JointConstPtr const > GetActiveJoints() const {
	return active_joints_;
}

[[nodiscard]] std::vector< std::string > GetJointNames() const {
	return joint_names_;
}

[[nodiscard]] std::vector< std::string > GetLinkNames() const {
	return link_names_;
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

const Link* GetJointParentLink( int i ) const {
	if ( i < 0 || i >= joints_.size() )
		throw std::out_of_range( "Invalid joint index" );
	return joints_[i]->GetParentLink();
}

const Link* const GetJointChildLink( int i ) const {
	if ( i < 0 || i >= joints_.size() )
		throw std::out_of_range( "Invalid joint index" );
	return joints_[i]->GetChildLink();
}

int GetJointIndex( const Joint* joint ) const;
const Joint* GetNextJoint( const Joint* joint ) const;
const Joint* GetPreviousJoint( const Joint* joint ) const;

[[nodiscard]] VecXd ActiveJointCenters() const noexcept {
	return ToVecXd( active_joint_centers_ );
}

[[nodiscard]] bool WithinLimits( const VecXd& joints ) const {
	if ( joints.size() > GetActiveJointCount() )
		return false;

	return WithinLimits( joints.data(), joints.size() );
}

[[nodiscard]] VecXd ClampLimits( const VecXd& joints ) const {
	if ( joints.size() > GetActiveJointCount() )
		throw std::invalid_argument( "Cannot clamp joints : invalid size." );

	return ClampLimits( joints.data(), joints.size() );
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

bool ComputeLinkPosesFK(
	const std::span< const double >& thetas,
	const Mat4d& home_configuration,
	std::vector< Mat4d >& links_fk,
	Mat4d& fk ) const
{
	return ComputeLinkPosesFK( thetas.data(), thetas.size(), GetLinkNames(), home_configuration, links_fk, fk );
}

bool ComputeLinkPosesFK(
	const VecXd& thetas,
	const Mat4d& home_configuration,
	std::vector< Mat4d >& links_fk,
	Mat4d& fk ) const
{
	return ComputeLinkPosesFK( thetas.data(), thetas.size(), GetLinkNames(), home_configuration, links_fk, fk );
}

bool ComputeLinkPosesFK(
	const std::span< const double >& thetas,
	const std::span< const std::string > link_names,
	const Mat4d& home_configuration,
	std::vector< Mat4d >& links_fk,
	Mat4d& fk ) const
{
	return ComputeLinkPosesFK( thetas.data(), thetas.size(), link_names, home_configuration, links_fk, fk );
}

bool ComputeLinkPosesFK(
	const VecXd& thetas,
	const std::span< const std::string > link_names,
	const Mat4d& home_configuration,
	std::vector< Mat4d >& links_fk,
	Mat4d& fk ) const
{
	return ComputeLinkPosesFK( thetas.data(), thetas.size(), link_names, home_configuration, links_fk, fk );
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
std::vector< LinkConstPtr > links_;
std::vector< JointConstPtr > active_joints_;

std::unordered_map< LinkConstPtr, JointConstPtr > parent_link_joint_map_;
std::unordered_map< LinkConstPtr, JointConstPtr > child_link_joint_map_;
std::unordered_map< std::string, JointConstPtr > joint_map_;
std::unordered_map< std::string, LinkConstPtr > link_map_;
std::vector< std::string > link_names_;
std::vector< std::string > joint_names_;

std::vector< double > active_joint_centers_;

JointChain( const std::vector< JointConstPtr >& joints, const std::vector< LinkConstPtr >& links );

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
	Mat4d& fk ) const;

bool ComputeLinkPosesFK(
	const double* thetas,
	int n_joints,
	const std::span< const std::string >& link_names,
	const Mat4d& home_configuration,
	std::vector< Mat4d >& links_fk,
	Mat4d& fk ) const noexcept;

bool WithinLimits( const double* joints, int n_joints ) const;
VecXd ClampLimits( const double* joints, int n_joints ) const;

friend class JointChainBuilder;
};

using JointChainConstPtr = std::shared_ptr< const JointChain >;
}