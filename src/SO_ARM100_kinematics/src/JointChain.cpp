#include "JointChain.hpp"

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
		return;

	joints_.emplace_back( std::move( joint ) );
	if ( joint->GetType() != JointType::FIXED )
		active_joints_.emplace_back( joint );
}

// ------------------------------------------------------------

[[nodiscard]] const JointChain JointChain::SubChain( int start_index, int count ) const
{
	const auto& joints = GetJoints();
	if ( start_index < 0 || count < 0 || start_index + count > static_cast< int >( joints.size() ) )
	{
		throw std::out_of_range( "Invalid subspan range" );
	}
	return JointChain( joints.subspan( start_index, count ) );
}

// ------------------------------------------------------------

}