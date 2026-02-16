#pragma once

#include "Global.hpp"

#include "BaseJointModel.hpp"
#include "IKinematicsSolver.hpp"
#include "Joint/Joint.hpp"

#include <span>

namespace SOArm100::Kinematics
{
class JointChain;

class BaseJointSolver : public IKinematicsSolver
{
public:
BaseJointSolver(
	std::shared_ptr< const JointChain > joint_chain,
	std::shared_ptr< const Mat4d > home_configuration,
	const BaseJointModel& base_joint_model );

virtual SolverResult IK(
	const Mat4d& target_pose,
	const std::span< const double >& seed_joints,
	double search_discretization ) const override;

void FK( const VecXd& base_joint, Mat4d& fk ) const;

private:
std::shared_ptr< const JointChain > joint_chain_;
std::shared_ptr< const Mat4d > home_configuration_;

BaseJointModelUniqueConstPtr base_joint_model_;

const Joint* GetBaseJoint() const;
};
}