#pragma once

#include "HybridSolver/BaseJointModel.hpp"
#include "IKHeuristic.hpp"
#include "Model/KinematicModel.hpp"

namespace SOArm100::Kinematics::Heuristic
{
class BaseJointHeuristic : public IKHeuristic
{
public:
BaseJointHeuristic( KinematicModelConstPtr model, const BaseJointModel& base_model );

virtual Solver::IKProblem Presolve( const Solver::IKProblem& problem ) const override;

private:
BaseJointModelUniqueConstPtr base_model_;
};
}