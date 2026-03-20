#pragma once

#include "Global.hpp"

#include "JointChain.hpp"
#include "KinematicTopoly.hpp"
#include "ReachableSpace.hpp"

#include <memory>

namespace SOArm100::Kinematics::Model
{
class KinematicModel
{
public:
KinematicModel(
	JointChainConstPtr chain,
	const Mat4d& home,
	const KinematicTopology& topology,
	ReachableSpaceUniqueConstPtr reachable_space ) :
	chain_( chain ),
	home_configuration_( home ),
	topology_( topology ),
	reachable_space_( std::move( reachable_space ) )
{
}

static KinematicModel Empty(){
	return KinematicModel( nullptr, Mat4d::Zero(), KinematicTopology(), nullptr );
}

bool IsEmpty() const {
	return !chain_ || home_configuration_.isZero();
}

bool ComputeFK( const VecXd& joints, Mat4d& fk ) const {
	if ( IsEmpty() )
		return false;
	return chain_->ComputeFK( joints, home_configuration_, fk );
}

const JointChain* GetChain() const {
	return chain_.get();
}

const Mat4d GetHomeConfiguration() const {
	return home_configuration_;
}

KinematicTopology GetTopology() const {
	return topology_;
}

bool IsUnreachable( const Mat4d& target ) const {
	if ( IsEmpty() )
		return true;
	return reachable_space_->IsUnreachable( target );
}

private:
JointChainConstPtr chain_;
Mat4d home_configuration_;
KinematicTopology topology_;
ReachableSpaceUniqueConstPtr reachable_space_;
};

using KinematicModelConstPtr = std::shared_ptr< const KinematicModel >;
}