#pragma once

#include "Heuristic/IKHeuristic.hpp"
#include "Model/JointGroup.hpp"

namespace SOArm100::Kinematics::Heuristic
{
class FabrikHeuristic : public IKHeuristic
{
public:
FabrikHeuristic( Model::KinematicModelConstPtr model, Model::JointGroup group );

virtual IKPresolution Presolve( const Solver::IKProblem& problem ) const override;

private:
Model::JointGroup group_;
};
}