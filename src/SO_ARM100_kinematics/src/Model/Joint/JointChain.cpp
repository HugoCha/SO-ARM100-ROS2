#include "Model/Joint/JointChain.hpp"

#include "Global.hpp"

#include "Model/Joint/Joint.hpp"
#include "Model/Joint/JointState.hpp"
#include "Model/Joint/Limits.hpp"
#include "Utils/KinematicsUtils.hpp"

#include <algorithm>
#include <stdexcept>

namespace SOArm100::Kinematics::Model
{

// ------------------------------------------------------------

JointChain::JointChain( int n )
{
	joints_.reserve( n );
	active_joints_.reserve( n );
}

// ------------------------------------------------------------

JointChain::JointChain( const std::span< JointConstPtr const >& joints )
{
	joints_.reserve( joints.size() );
	active_joints_.reserve( joints.size() );
	for ( const auto& joint : joints )
	{
		Add( joint );
	}
}

// ------------------------------------------------------------

int JointChain::GetJointIndex( const Joint* joint ) const
{
	if ( Empty() || !joint )
		return -1;

	auto it = std::find_if( joints_.begin(), joints_.end(),
	                        [joint]( const JointConstPtr& ptr ){
			return ptr.get() == joint;
		} );

	if ( it != joints_.end() )
	{
		return std::distance( joints_.begin(), it );
	}

	return -1;
}

// ------------------------------------------------------------

const Joint* JointChain::GetNextJoint( const Joint* joint ) const
{
	auto index = GetJointIndex( joint );

	if ( index >= 0 && index + 1 < joints_.size() )
	{
		return joints_[index + 1].get();
	}

	return nullptr;
}

// ------------------------------------------------------------

const Joint* JointChain::GetPreviousJoint( const Joint* joint ) const
{
	auto index = GetJointIndex( joint );

	if ( index >= 1 )
	{
		return joints_[index - 1].get();
	}

	return nullptr;
}

// ------------------------------------------------------------

bool JointChain::WithinLimits( const double* joints, int n_joints ) const
{
	n_joints = std::min( n_joints, ( int )GetActiveJointCount() );

	for ( size_t i = 0; i < n_joints; i++ )
		if ( !GetActiveJointLimits( i ).Within( joints[i] ) )
			return false;

	return true;
}

// ------------------------------------------------------------

VecXd JointChain::ClampLimits( const double* joints, int n_joints ) const
{
	n_joints = std::min( n_joints, ( int )GetActiveJointCount() );
	VecXd clamped( n_joints );

	for ( size_t i = 0; i < n_joints; i++ )
		clamped[i] = GetActiveJointLimits( i ).Clamp( joints[i] );

	return clamped;
}

// ------------------------------------------------------------

VecXd JointChain::RandomValidJoints(
	random_numbers::RandomNumberGenerator& rng,
	double margin_percent ) const noexcept
{
	const int n_joints = GetActiveJointCount();
	VecXd random( n_joints );
	for ( int i = 0; i < n_joints; i++ )
	{
		auto joint = active_joints_[i];
		const auto& limits = joint->GetLimits();
		limits.Random( rng, & random.data()[i], margin_percent );
	}
	return random;
}

// ------------------------------------------------------------

VecXd JointChain::RandomValidJointsNear(
	random_numbers::RandomNumberGenerator& rng,
	const VecXd& joints,
	double distance,
	double margin_percent ) const noexcept
{
	const int n_joints = GetActiveJointCount();
	VecXd random( n_joints );
	for ( int i = 0; i < n_joints; i++ )
	{
		auto joint = active_joints_[i];
		const auto& limits = joint->GetLimits();
		limits.RandomNear(
			rng,
			joints[i],
			& random.data()[i],
			distance,
			margin_percent );
	}
	return random;
}

// ------------------------------------------------------------

VecXd JointChain::RandomValidJointsNearWrapped(
	random_numbers::RandomNumberGenerator& rng,
	const VecXd& joints,
	double min_limit_span,
	double distance,
	double margin_percent ) const noexcept
{
	const int n_joints = GetActiveJointCount();
	VecXd random( n_joints );
	for ( size_t i = 0; i < n_joints; i++ )
	{
		auto joint = active_joints_[i];
		const auto& limit = joint->GetLimits();
		if ( joint->IsRevolute() && limit.Span() >= min_limit_span )
		{
			limit.RandomNearWrapped( rng,
			                         joints[i],
			                         & random.data()[i],
			                         distance );
		}
		else
		{
			limit.RandomNear( rng,
			                  joints[i],
			                  & random.data()[i],
			                  distance,
			                  margin_percent );

		}
	}
	return random;
}

// ------------------------------------------------------------

VecXd JointChain::RandomValidJointsNearCentered(
	random_numbers::RandomNumberGenerator& rng,
	const VecXd& joints,
	double distance,
	double margin_percent ) const noexcept
{
	const int n_joints = GetActiveJointCount();
	VecXd random( n_joints );
	for ( size_t i = 0; i < n_joints; i++ )
	{
		const auto& limits = GetActiveJointLimits( i );
		double target = joints[i];
		double margin = margin_percent * limits.Span();
		double min_tol = limits.Min() + margin;
		double max_tol = limits.Max() - margin;
		if ( target < min_tol || target > max_tol )
		{
			target = limits.Center();
		}

		limits.RandomNear(
			rng,
			target,
			& random.data()[i],
			distance,
			margin_percent );
	}
	return random;
}

// ------------------------------------------------------------

bool JointChain::ComputeFK(
	const double* thetas,
	int n_joints,
	const Mat4d& home_configuration,
	Mat4d& fk ) const noexcept
{
	n_joints = std::min( n_joints, ( int )GetActiveJointCount() );

	if ( !WithinLimits( thetas, n_joints ) )
		return false;

	Mat4d T_cumul = Mat4d::Identity();

	for ( int i = 0; i < n_joints; i++ )
	{
		const auto& twist = GetActiveJointTwist( i );
		T_cumul *= twist.ExponentialMatrix( thetas[i] );
	}

	fk.noalias() = T_cumul * home_configuration;
	return true;
}

// ------------------------------------------------------------

bool JointChain::ComputeJointStatesFK(
	const double* thetas,
	int n_joints,
	const Mat4d& home_configuration,
	std::vector< JointState >& joint_states,
	Mat4d& fk ) const
{
	const int n_model_joints = GetActiveJointCount();
	fk.setIdentity();

	if ( n_joints != n_model_joints )
		throw std::invalid_argument( "Joints size mismatch" );

	if ( joint_states.size() != n_model_joints )
		throw std::invalid_argument( "Joints state size mismatch" );

	if ( !WithinLimits( thetas, n_joints ) )
		return false;

	Mat4d T_cumul = Mat4d::Identity();

	for ( size_t i = 0; i < n_joints; i++ )
	{
		const auto& joint = GetActiveJoint( i );

		auto joint_pose  = joint->Pose( T_cumul );
		joint_states[i].Origin() = joint_pose.origin;
		joint_states[i].Axis() = joint_pose.axis;
		joint_states[i].Value() = thetas[i];

		const auto& twist = joint->GetTwist();
		T_cumul *= twist.ExponentialMatrix( thetas[i] );
	}

	fk.noalias() = T_cumul * home_configuration;
	return true;
}

// ------------------------------------------------------------

bool JointChain::ComputeJointPosesFK(
	const double* thetas,
	int n_joints,
	const std::span< const std::string >& link_names,
	const Mat4d& home_configuration,
	std::vector< Mat4d >& joint_poses,
	Mat4d& fk ) const noexcept
{
	fk.setIdentity();

	if ( n_joints != GetActiveJointCount() )
		return false;

	if ( !WithinLimits( thetas, n_joints ) )
		return false;

	if ( joint_poses.size() != link_names.size() )
		joint_poses.resize( link_names.size() );

	Mat4d T_cumul = Mat4d::Identity();

	std::vector< Mat4d > all_poses( n_joints );
	std::map< std::string, int > link_pose_index;
	std::set< std::string > link_names_set( link_names.begin(), link_names.end() );

	for ( size_t i = 0; i < n_joints; i++ )
	{
		const auto& joint = GetActiveJoint( i );
		const auto& link = joint->GetLink();
		const auto& twist = joint->GetTwist();

		if ( link_names_set.contains( link.GetName() ) )
			link_pose_index[link.GetName()] = i;

		all_poses[i].noalias() = T_cumul * joint->OriginTransform();
		T_cumul *= twist.ExponentialMatrix( thetas[i] );
	}

	for ( int i = 0; i < link_names.size(); i++ )
		joint_poses[i] = all_poses[link_pose_index[link_names[i]]];

	fk.noalias() = T_cumul * home_configuration;
	return true;
}

// ------------------------------------------------------------

void JointChain::Add( JointConstPtr joint )
{
	if ( !joint )
		throw std::invalid_argument( "Joint pointer cannot be null" );

	joints_.emplace_back( joint );
	if ( !joint->IsFixed() )
	{
		active_joints_.emplace_back( joint );
		active_joint_centers_.emplace_back( joint->GetLimits().Center() );
	}
}

// ------------------------------------------------------------

JointChain JointChain::SubChain( JointConstPtr start, JointConstPtr end ) const
{
	const auto& joints = GetJoints();

	auto start_it = std::find( joints.begin(), joints.end(), start );
	auto end_it = std::find( joints.begin(), joints.end(), end );

	if ( start_it == joints.end() || end_it == joints.end() )
	{
		throw std::out_of_range( "Invalid subchain link" );
	}

	size_t start_index = static_cast< size_t >( std::distance( joints.begin(), start_it ) );
	size_t end_index = static_cast< size_t >( std::distance( joints.begin(), end_it ) );

	if ( start_index > end_index )
	{
		throw std::out_of_range( "Start index must be less than or equal to end index" );
	}

	size_t count = end_index - start_index + 1;

	return JointChain( joints.subspan( start_index, count ) );
}

// ------------------------------------------------------------

}