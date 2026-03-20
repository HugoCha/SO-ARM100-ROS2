#pragma once

#include "Global.hpp"
#include "Model/JointGroup.hpp"
#include "Model/KinematicModel.hpp"
#include "Heuristic/IKHeuristic.hpp"

namespace SOArm100::Kinematics::Heuristic
{
class BaseJointHeuristic : public IKHeuristic
{
public:
BaseJointHeuristic(
	Model::KinematicModelConstPtr model,
	const Model::JointGroup& base_group );

virtual IKPresolution Presolve( 
    const Solver::IKProblem& problem, 
    const Solver::IKRunContext& context ) const override;

private:
Model::JointGroup base_group_;
Vec3d reference_direction_;

const Model::Joint* GetBaseJoint() const;
Vec3d ComputeReferenceDirection() const;
Vec3d TipHomePosition() const;
Vec3d ComputeTipPosition( const Mat4d& target ) const;
};
}