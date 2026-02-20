#include "HybridSolver/BaseWristSolver.hpp"

#include "Global.hpp"
#include "HybridSolver/BaseJointSolver.hpp"
#include "HybridSolver/NumericJointsSolver.hpp"
#include "HybridSolver/WristSolver.hpp"
#include "Utils/KinematicsUtils.hpp"
#include "SolverResult.hpp"

#include <memory>

namespace SOArm100::Kinematics
{

// ------------------------------------------------------------

BaseWristSolver::BaseWristSolver(
	std::shared_ptr< const JointChain > joint_chain,
	std::shared_ptr< const Mat4d > home_configuration,
	const BaseJointModel& base_model,
	const WristModel& wrist_model ) :
	joint_chain_( joint_chain ),
	home_configuration_( home_configuration ),
	buffer_( SolverBuffer( 1, wrist_model.active_joint_count ) )
{
	base_joint_solver_ = std::make_unique< BaseJointSolver >( joint_chain, home_configuration, base_model );
	wrist_solver_ = std::make_unique< WristSolver >( joint_chain, home_configuration, wrist_model );
}

// ------------------------------------------------------------

SolverResult BaseWristSolver::IK(
	const Mat4d& target_pose,
	const std::span< const double >& seed_joints,
	double discretization ) const
{
	SolverResult result( buffer_.Size() );

	wrist_solver_->ComputeWristCenter( target_pose, buffer_.wrist_center );

	if ( ( buffer_.base_result = base_joint_solver_->IK(
			   buffer_.wrist_center,
			   seed_joints,
			   discretization ) ).Unreachable() )
	{
		result.state = SolverState::Unreachable;
		return result;
	}

	base_joint_solver_->FK( buffer_.base_result.joints, buffer_.T_base );
	buffer_.wrist_target = Inverse( buffer_.T_base ) * target_pose;

	if ( ( buffer_.wrist_result = wrist_solver_->IK(
			   buffer_.wrist_target,
			   seed_joints,
			   discretization ) ).Unreachable() )
	{
		result.state = SolverState::Unreachable;
		return result;
	}

	result.state = GetSolverState(
		{
			buffer_.base_result,
			buffer_.wrist_result
		} );

	result.joints << buffer_.base_result.joints,
	    buffer_.wrist_result.joints;

	CheckSolverResult(
		*joint_chain_,
		*home_configuration_,
		target_pose,
		buffer_.fk_result,
		result );

	return result;
}

// ------------------------------------------------------------

}