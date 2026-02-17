#include "Joint/JointChain.hpp"

#include "Joint/Joint.hpp"

#include <algorithm>

namespace SOArm100::Kinematics
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

int JointChain::GetJointIndex( const JointConstPtr& joint ) const
{
	if ( Empty() || !joint ) 
		return -1;

	auto it = std::ranges::find( joints_, joint );

	if ( it != joints_.end() )
	{
		return std::distance( joints_.begin(), it );
	}

	return -1;
}

// ------------------------------------------------------------

const Joint* JointChain::GetNextJoint( const JointConstPtr& joint ) const
{
	auto index = GetJointIndex( joint );

	if ( index >= 0 && index + 1 < joints_.size() )
	{
		return joints_[ index + 1 ].get();
	}

	return nullptr;
}

// ------------------------------------------------------------

const Joint* JointChain::GetPreviousJoint( const JointConstPtr& joint ) const
{
	auto index = GetJointIndex( joint );

	if ( index >= 1 )
	{
		return joints_[ index - 1 ].get();
	}

	return nullptr;
}

// ------------------------------------------------------------

void JointChain::Add( JointConstPtr joint )
{
	if ( !joint )
		throw std::invalid_argument( "Joint pointer cannot be null" );

	joints_.emplace_back( joint );
	if ( !joint->IsFixed() )
	{
		active_joints_to_joints_map_.emplace( joint, active_joints_.size() );
		active_joints_.emplace_back( joint );
	}
}

// ------------------------------------------------------------

const JointChain JointChain::SubChain( JointConstPtr start, JointConstPtr end ) const
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