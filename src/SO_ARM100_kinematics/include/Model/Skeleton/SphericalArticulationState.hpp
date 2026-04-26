#pragma once

#include "ArticulationState.hpp"
#include "Model/Skeleton/BoneState.hpp"
#include "Utils/Euler.hpp"

namespace SOArm100::Kinematics::Model
{
class SphericalArticulationState : public ArticulationState
{
public:
SphericalArticulationState( ArticulationConstPtr articulation );

virtual void ApplyConstraints( BoneState& bone_state ) const override;
virtual void UpdateValues( const BoneState& bone_state ) override;

private:
std::optional< Euler::Configuration > euler_configuration_;
};
}