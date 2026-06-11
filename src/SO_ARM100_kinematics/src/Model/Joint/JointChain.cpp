#include "Model/Joint/JointChain.hpp"

#include "Global.hpp"

#include "Model/Joint/Joint.hpp"
#include "Model/Joint/JointState.hpp"
#include "Model/Joint/Limits.hpp"
#include "Model/Joint/Link.hpp"
#include "Utils/KinematicsUtils.hpp"

#include <algorithm>
#include <stdexcept>
#include <vector>

namespace SOArm100::Kinematics::Model
{

// ------------------------------------------------------------

JointChain::JointChain( const std::vector< JointConstPtr >& joints, 
						const std::vector< LinkConstPtr >& links )
{
	if ( links.size() != joints.size() + 1 )
		throw std::invalid_argument( "Link must have size joints size + 1" );

	joints_.resize( joints.size() );
	links_.resize( links.size() );
	joint_names_.resize( joints.size() );
	link_names_.resize( links.size() );

	for ( int i = 0; i < joints.size(); i++ )
	{
		joints_[i] = joints[i];
		links_[i]  = links[i];

		const auto& joint = joints_[i];
		const auto& parent_link = links_[i];
		const auto& child_link = links[i+1];
		
		if ( joint->GetParentLink() != parent_link.get() || joint->GetChildLink() != child_link.get() )
			throw std::invalid_argument( "Joint/Link mismatch" );
	
		link_names_[i] = parent_link->GetName();
		link_map_.emplace( std::make_pair( parent_link->GetName(), parent_link ) );
		parent_link_joint_map_.emplace( std::make_pair( parent_link, joint ) );
		child_link_joint_map_.emplace( std::make_pair( child_link, joint ) );

		if ( !joint->IsFixed() ) 
		{
			active_joints_.emplace_back( joint );
			active_joint_centers_.emplace_back( joint->GetLimits().Center() );
		}

		joint_names_[i] = joint->GetName();
		joint_map_.emplace( std::make_pair( joint->GetName(), joint ) );
	}

	if ( !joints.empty() )
	{
		int tip_link_index = joints.size();

		auto tip_link = links[tip_link_index];

		link_names_[tip_link_index] = tip_link->GetName();
		link_map_.emplace( std::make_pair( tip_link->GetName(), tip_link ) );
		links_[tip_link_index] = tip_link;
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
	fk.setIdentity();
	
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

bool JointChain::ComputeLinkPosesFK(
	const double* thetas,
	int n_joints,
	const std::span< const std::string >& link_names,
	const Mat4d& home_configuration,
	std::vector< Mat4d >& links_fk,
	Mat4d& fk ) const noexcept
{
	fk.setIdentity();

	if ( n_joints != GetActiveJointCount() )
		return false;

	if ( !WithinLimits( thetas, n_joints ) )
		return false;

	if ( links_fk.size() != link_names.size() )
		links_fk.resize( link_names.size() );

	std::map< std::string, Mat4d > joint_poses;
	Mat4d T_cumul = Mat4d::Identity();

	for ( size_t i = 0; i < n_joints; i++ )
	{
		const auto& joint = GetActiveJoint( i );
		const auto& twist = joint->GetTwist();

		auto joint_pose = T_cumul * joint->OriginTransform();
		T_cumul *= twist.ExponentialMatrix( thetas[i] );
		joint_poses.emplace( std::make_pair( joint->GetName(), joint_pose ) );
	}

	for ( int i = 0; i < link_names.size(); i++ )
	{
		if ( link_map_.contains( link_names[i] ) )
		{
			auto link = link_map_.at( link_names[i] );
			if ( child_link_joint_map_.contains( link ) )
			{
				auto parent_joint = child_link_joint_map_.at( link );
				auto parent_joint_name = parent_joint->GetName();
				links_fk[i] = joint_poses.at( parent_joint_name ) * link->ParentJointTransform();
			}
			else 
			{
				// Root Link
				links_fk[i] = link->HomeTransform();
			}
		}
	}

	fk.noalias() = T_cumul * home_configuration;
	return true;
}

// ------------------------------------------------------------

JointChain JointChain::SubChain( JointConstPtr start, JointConstPtr end ) const
{
	const auto& joints = GetJoints();
	const auto& links = GetLinks();

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

	auto sub_joints = joints.subspan( start_index, count );
	auto sub_links = links.subspan( start_index, count + 1 );
	return JointChain( 
		std::vector< JointConstPtr >( sub_joints.begin(), sub_joints.end() ), 
		std::vector< LinkConstPtr >( sub_links.begin(), sub_links.end() ) );
}

// ------------------------------------------------------------

}