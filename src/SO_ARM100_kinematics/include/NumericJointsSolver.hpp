#pragma once

#include "DLSKinematicsSolver.hpp"
#include "NumericJointsModel.hpp"

#include <memory>

namespace SOArm100::Kinematics
{
struct NumericSolverResult;

class NumericJointsSolver
{
public:
NumericJointsSolver();

void Initialize(
	const JointChain& joint_chain,
	const NumericJointsModel& numeric_joint_model,
	double search_discretization );

[[nodiscard]] NumericSolverResult IK(
	const Mat4d& target_pose,
	const std::span< const double >& seed_joints ) const;

bool FK( const VecXd& joints, Mat4d& fk ) const;

private:
std::unique_ptr< DLSKinematicsSolver > dls_solver_;
NumericJointsModelUniqueConstPtr numeric_joints_model_;
};
}