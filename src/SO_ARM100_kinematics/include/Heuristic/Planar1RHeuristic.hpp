#pragma once

#include "Global.hpp"

#include "Heuristic/IIKHeuristic.hpp"
#include "Model/IKJointGroupModelBase.hpp"

namespace SOArm100::Kinematics::Heuristic
{
class Joint;

class Planar1RHeuristic : public Model::IKJointGroupModelBase, public IIKHeuristic
{
public:
Planar1RHeuristic(
	Model::KinematicModelConstPtr model,
	Model::JointGroup planar_group );

virtual IKPresolution Presolve(
	const Solver::IKProblem& problem,
	const Solver::IKRunContext& context ) const override;

private:
Vec3d reference_direction_;

double L() const;
Model::JointConstPtr GetJoint() const;

static Vec3d ComputeReferenceDirection(
	Model::KinematicModelConstPtr model,
	const Model::JointGroup planar_group );
};
}