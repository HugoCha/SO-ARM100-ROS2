#pragma once

#include "Global.hpp"
#include "Heuristic/JointGroupHeuristic.hpp"
#include "Model/JointGroup.hpp"
#include "Model/KinematicModel.hpp"

namespace SOArm100::Kinematics::Heuristic
{
class RevoluteBaseHeuristic : public JointGroupHeuristic
{
public:
RevoluteBaseHeuristic(
	Model::KinematicModelConstPtr model,
	const Model::RevoluteBaseJointGroup& revolute_base_group );

virtual IKPresolution Presolve(
	const Solver::IKProblem& problem,
	const Solver::IKRunContext& context ) const override;

private:
Vec3d reference_direction_;

const Model::Joint* GetBaseJoint() const;
Vec3d ComputeReferenceDirection() const;
};
}