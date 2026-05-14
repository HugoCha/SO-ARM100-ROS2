#pragma once

#include "ArticulationState.hpp"
#include "Model/Skeleton/BoneState.hpp"
#include "Euler/SphericalSolver.hpp"
#include <memory>

namespace SOArm100::Kinematics::Model
{
class SphericalArticulationState : public ArticulationState
{
public:
SphericalArticulationState( const Articulation* articulation );

virtual void ApplyConstraints( BoneState& bone_state ) const override;
virtual void UpdateValues( const BoneState& bone_state ) override;

private:
std::unique_ptr< Solver::SphericalSolver > spherical_solver_;
};
}