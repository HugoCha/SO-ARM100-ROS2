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

void JointChain::Add( JointConstPtr joint )
{
	if ( !joint )
		throw std::invalid_argument( "Joint pointer cannot be null" );

	joints_.emplace_back( joint );
	if ( joint->GetType() != JointType::FIXED )
		active_joints_.emplace_back( joint );
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