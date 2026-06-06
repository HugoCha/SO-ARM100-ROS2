#pragma once

#include "ArticulationState.hpp"
#include "Model/Skeleton/BoneState.hpp"
#include "SphericalSolver/SphericalSolver.hpp"

#include <memory>

namespace SOArm100::Kinematics::Model
{
class SphericalArticulationState : public ArticulationState
{
public:
SphericalArticulationState( const Articulation* articulation );

virtual void ApplyConstraints( BoneState& bone_state ) const override;
virtual void UpdateValues( 
    const VecXd& seed,
    const BoneState& bone_state, 
    double damping_factor = 1.0 ) override;

private:
std::unique_ptr< Solver::SphericalSolver > solver_;
};
}