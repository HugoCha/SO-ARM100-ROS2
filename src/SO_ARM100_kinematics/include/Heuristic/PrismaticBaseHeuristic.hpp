#pragma once

#include "Heuristic/IIKHeuristic.hpp"
#include "Model/IKJointGroupModelBase.hpp"

namespace SOArm100::Kinematics::Heuristic
{
class PrismaticBaseHeuristic :
	public Model::IKJointGroupModelBase,
	public IIKHeuristic
{
public:
PrismaticBaseHeuristic(
	Model::KinematicModelConstPtr model,
	const Model::JointGroup& prismatic_base_group );

virtual IKPresolution Presolve(
	const Solver::IKProblem& problem,
	const Solver::IKRunContext& context ) const override;

private:
const Model::Joint* GetBaseJoint() const;
};
}