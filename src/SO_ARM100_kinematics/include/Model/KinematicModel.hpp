#pragma once

#include "Global.hpp"
#include "JointChain.hpp"
#include "Model/KinematicTopoly.hpp"
#include <memory>

namespace SOArm100::Kinematics::Model
{
class KinematicModel
{
public:
KinematicModel( JointChainConstPtr chain, const Mat4d& home, const KinematicTopology& topology ) :
    chain_( chain ),
    home_configuration_( home ),
    topology_( topology )
{}

KinematicModel( const KinematicModel& model ) :
    KinematicModel( model.chain_, model.home_configuration_, model.topology_ )
{}

static KinematicModel Empty() {
    return KinematicModel( nullptr, Mat4d::Zero(), {} );
}

bool IsEmpty() const {
    return !chain_ || home_configuration_.isZero();
}

bool ComputeFK( const VecXd& joints, Mat4d& fk ) const {
    if ( IsEmpty() ) return false;
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

private:
JointChainConstPtr chain_;
Mat4d home_configuration_;
KinematicTopology topology_;
};

using KinematicModelConstPtr = std::shared_ptr< const KinematicModel >;
}