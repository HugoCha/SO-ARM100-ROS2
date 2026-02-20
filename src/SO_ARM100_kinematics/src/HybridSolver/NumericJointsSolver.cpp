#include "HybridSolver/NumericJointsSolver.hpp"

#include "DLSSolver/DLSKinematicsSolver.hpp"
#include "DLSSolver/NumericSolverResult.hpp"
#include "Global.hpp"

#include <memory>

namespace SOArm100::Kinematics
{

// ------------------------------------------------------------

NumericJointsSolver::NumericJointsSolver(
	std::shared_ptr< const JointChain > joint_chain,
	std::shared_ptr< const Mat4d > home_configuration,
	const NumericJointsModel& numeric_joint_model,
	SolverType type )
{
	numeric_joints_model_ = std::make_unique< const NumericJointsModel >( numeric_joint_model );

	const auto& active_joints = joint_chain->GetActiveJoints();
	auto start = active_joints[numeric_joint_model.start_index];
	auto end = active_joints[numeric_joint_model.start_index + numeric_joint_model.count - 1];

	if ( start == active_joints.front() && end == active_joints.back() )
	{
		joint_chain_ = joint_chain;
		home_configuration_ = home_configuration;
	}
	else
	{
		joint_chain_ = std::make_shared< const JointChain >( joint_chain->SubChain( start, end ) );
		home_configuration_ = std::make_shared< const Mat4d >( numeric_joint_model.home_configuration );
	}

	dls_solver_ = std::make_unique< DLSKinematicsSolver >();
	dls_solver_->Initialize(
		joint_chain_,
		home_configuration_,
		0.01 );
}

// ------------------------------------------------------------

bool NumericJointsSolver::FK( const VecXd& joints, Mat4d& fk ) const
{
	return dls_solver_->ForwardKinematic( joints, fk );
}

// ------------------------------------------------------------

SolverResult NumericJointsSolver::IK(
	const Mat4d& target_pose,
	const std::span< const double >& seed_joints,
	double search_discretization ) const
{
	return ToSolverResult( dls_solver_->InverseKinematic(
							target_pose,
							seed_joints.subspan( numeric_joints_model_->start_index, numeric_joints_model_->count ) ) );
}

// ------------------------------------------------------------

}