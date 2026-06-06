#pragma once

#include "ArticulationState.hpp"
#include "UniversalSolver/UniversalSolver.hpp"

namespace SOArm100::Kinematics::Solver
{
    struct UniversalSolution;
}

namespace SOArm100::Kinematics::Model
{
class Limits;
class UniversalArticulationState : public ArticulationState
{
public:
UniversalArticulationState( const Articulation* articulation );

virtual void ApplyConstraints( BoneState& bone_state ) const override;
virtual void UpdateValues(
    const VecXd& seed, 
    const BoneState& bone_state, 
    double damping_factor = 1.0 ) override;

private:
std::unique_ptr< Solver::UniversalSolver > solver_;
};
}