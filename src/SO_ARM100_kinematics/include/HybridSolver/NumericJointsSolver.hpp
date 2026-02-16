#pragma once

#include "DLSSolver/DLSKinematicsSolver.hpp"
#include "IKinematicsSolver.hpp"
#include "NumericJointsModel.hpp"

#include <memory>

namespace SOArm100::Kinematics
{
struct SolverResult;

class NumericJointsSolver : public IKinematicsSolver
{
public:
NumericJointsSolver(
	const JointChain& joint_chain,
	const NumericJointsModel& numeric_joint_model );

virtual SolverResult IK(
	const Mat4d& target_pose,
	const std::span< const double >& seed_joints,
	double search_discretization ) const override;

bool FK( const VecXd& joints, Mat4d& fk ) const;

private:
std::unique_ptr< DLSKinematicsSolver > dls_solver_;
NumericJointsModelUniqueConstPtr numeric_joints_model_;
};
}