#include "NumericJointsSolver.hpp"

#include "NumericSolverResult.hpp"

namespace SOArm100::Kinematics
{

// ------------------------------------------------------------

NumericJointsSolver::NumericJointsSolver() :
	dls_solver_()
{
}

// ------------------------------------------------------------

void NumericJointsSolver::Initialize(
	const JointChain& joint_chain,
	const NumericJointsModel& numeric_joint_model,
	double search_discretization )
{
	numeric_joints_model_ = std::make_unique< const NumericJointsModel >( numeric_joint_model );
	dls_solver_->Initialize(
		joint_chain.SubChain( numeric_joint_model.start_index, numeric_joint_model.count ),
		numeric_joint_model.home_configuration,
		search_discretization );
}

// ------------------------------------------------------------

bool NumericJointsSolver::FK( const VecXd& joints, Mat4d& fk ) const
{
	return dls_solver_->ForwardKinematic( joints, fk );
}

// ------------------------------------------------------------

NumericSolverResult NumericJointsSolver::IK(
	const Mat4d& target_pose,
	const std::span< const double >& seed_joints ) const
{
	return dls_solver_->SolveIK(
		target_pose,
		seed_joints.subspan( numeric_joints_model_->start_index, numeric_joints_model_->count ) );
}

// ------------------------------------------------------------

}