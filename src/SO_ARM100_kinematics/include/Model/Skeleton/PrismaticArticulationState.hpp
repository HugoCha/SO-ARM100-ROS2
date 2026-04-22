#pragma once

#include "ArticulationState.hpp"

namespace SOArm100::Kinematics::Model
{
class PrismaticArticulationState : public ArticulationState
{
public:
PrismaticArticulationState( ArticulationConstPtr articulation );

virtual void ApplyConstraints( BoneState& bone_state ) const override;
virtual void UpdateValues( const BoneState& bone_state ) override;
};
}