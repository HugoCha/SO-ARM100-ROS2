#pragma once

#include "Global.hpp"

#include "Heuristic/IIKHeuristic.hpp"
#include "Model/IKJointGroupModelBase.hpp"

namespace SOArm100::Kinematics::Heuristic
{
class RevoluteBaseHeuristic : public Model::IKJointGroupModelBase, IIKHeuristic
{
public:
RevoluteBaseHeuristic(
	Model::KinematicModelConstPtr model,
	const Model::JointGroup& revolute_base_group );

virtual IKPresolution Presolve(
	const Solver::IKProblem& problem,
	const Solver::IKRunContext& context ) const override;

private:
Vec3d reference_direction_;

const Model::Joint* GetBaseJoint() const;
Vec3d ComputeReferenceDirection() const;
Vec3d ComputeDirection( const Mat4d& T_tip ) const;
};
}