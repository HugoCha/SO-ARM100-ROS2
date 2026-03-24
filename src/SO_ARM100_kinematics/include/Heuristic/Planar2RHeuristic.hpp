#pragma once

#include "Global.hpp"

#include "Heuristic/IIKHeuristic.hpp"
#include "Model/IKJointGroupModelBase.hpp"

namespace SOArm100::Kinematics::Heuristic
{
class Planar2RHeuristic : public Model::IKJointGroupModelBase, IIKHeuristic
{
public:
Planar2RHeuristic(
	Model::KinematicModelConstPtr model,
	Model::JointGroup planar_group );

virtual IKPresolution Presolve(
	const Solver::IKProblem& problem,
	const Solver::IKRunContext& context ) const override;

private:
Vec3d reference_direction_;
Vec3d up_direction_;

static Vec3d ComputeReferenceDirection(
	Model::KinematicModelConstPtr model,
	const Model::JointGroup planar_group );

static Vec3d ComputeUpDirection(
	const Vec3d& reference_direction,
	Model::KinematicModelConstPtr model,
	const Model::JointGroup planar_group );

double L1() const;
double L2() const;

bool ValidateAndSelectElbowConfiguration(
	const VecXd& planar_seed,
	const VecXd& elbow_up,
	const VecXd& elbow_down,
	VecXd& solution ) const;
};
}