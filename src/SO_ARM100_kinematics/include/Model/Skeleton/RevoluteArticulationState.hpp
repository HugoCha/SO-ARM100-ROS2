#pragma once

#include "ArticulationState.hpp"

namespace SOArm100::Kinematics::Model
{
class RevoluteArticulationState : public ArticulationState
{
public:
RevoluteArticulationState( const Articulation* articulation );

virtual void ApplyConstraints( BoneState& bone_state ) const override;

virtual void UpdateValues( 
    const VecXd& seed, 
    const BoneState& bone_state, 
    double damping_factor = 1.0 ) override;
};
}