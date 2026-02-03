#include "DLSKinematicsSolver.hpp"

#include "Converter.hpp"
#include "KinematicsUtils.hpp"
#include "Types.hpp"

#include <Eigen/Dense>
#include <Eigen/src/SVD/JacobiSVD.h>
#include <moveit/robot_model/joint_model.hpp>
#include <moveit/robot_model/link_model.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <optional>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <vector>

namespace SOArm100::Kinematics
{

// ------------------------------------------------------------

static rclcpp::Logger get_logger()
{
	static rclcpp::Logger logger = rclcpp::get_logger( "DLSKinematicsSolver" );
	return logger;
}

// ------------------------------------------------------------

DLSKinematicsSolver::DLSKinematicsSolver() :
	SOArm100::Kinematics::DLSKinematicsSolver( SolverParameters{} )
{
}

// ------------------------------------------------------------

DLSKinematicsSolver::DLSKinematicsSolver( SolverParameters parameters )
	: parameters_( parameters ), buffers_( twists_.size() )
{
	if ( !parameters_.IsValid() )
	{
		throw std::invalid_argument( "Invalid DLS configuration" );
	}
}

// ------------------------------------------------------------

DLSKinematicsSolver::IKResult DLSKinematicsSolver::SolveIK(
	const geometry_msgs::msg::Pose& target_pose,
	const std::span< const double >& seed_joints ) const
{
	const Mat4d target = ToMat4d( target_pose );

	if ( buffers_.GetSize() != static_cast< int >( twists_.size() ) )
	{
		buffers_ = SolverBuffers{ twists_.size() };
	}

	auto state = InitializeState( target, ToVecXd( seed_joints ) );
	if ( !state )
	{
		state = InitializeState( target, RandomValidJoints() );
		if ( !state )
		{
			return { SolverState::Failed, VecXd{}, 0.0, 0 };
		}
	}

	for ( int iter = 0; iter < parameters_.max_iterations; ++iter )
	{
		auto solver_state = EvaluateConvergence( *state, iter );
		if ( solver_state == SolverState::Converged )
		{
			return { SolverState::Converged, state->joints, state->error, iter };
		}

		if ( IsStalled( *state ) )
		{
			state = InitializeState( target, RandomValidJoints() );
			if ( !state )
			{
				return { SolverState::Stalled, VecXd{}, 0.0, iter };
			}
			continue;
		}

		PerformIteration( target, *state, buffers_ );
	}

	return { SolverState::MaxIterations, state->joints, state->error,
	         parameters_.max_iterations };
}

// ------------------------------------------------------------

bool DLSKinematicsSolver::InverseKinematic(
	const geometry_msgs::msg::Pose& target_pose,
	const std::span< const double >& seed_joints,
	std::vector< double >& joint_angles ) const
{
	auto result = SolveIK( target_pose, seed_joints );

	if ( result.Success() )
	{
		joint_angles = ToStdVector( result.joint_angles );
		return true;
	}

	return false;
}

// ------------------------------------------------------------

std::optional< DLSKinematicsSolver::IterationState > DLSKinematicsSolver::InitializeState(
	const Mat4d& target,
	const VecXd& initial_joints ) const
{
	if ( initial_joints.size() != static_cast< int >( twists_.size() ) )
	{
		RCLCPP_ERROR( get_logger(), "InitializeState: Joint vector size mismatch." );
		return std::nullopt;
	}

	IterationState state;
	state.joints = initial_joints;

	if ( !ForwardKinematic( state.joints, buffers_.fk ) )
	{
		RCLCPP_ERROR( get_logger(), "InitializeState: Forward Kinematics failed." );
		return std::nullopt;
	}

	PoseError( target, buffers_.fk, buffers_.error );

	state.error = buffers_.error.squaredNorm();
	state.step = parameters_.max_step;
	state.damping = parameters_.min_damping;
	state.fk_failures = 0;

	return state;
}

// ------------------------------------------------------------

const VecXd DLSKinematicsSolver::RandomValidJoints() const noexcept
{
	assert( joint_model_ != nullptr );
	VecXd random( twists_.size() );
	random_numbers::RandomNumberGenerator rng;
	joint_model_->getVariableRandomPositions( rng, random.data() );
	return random;
}

// ------------------------------------------------------------

void DLSKinematicsSolver::PerformIteration(
	const Mat4d& target,
	IterationState& state,
	SolverBuffers& buffers ) const
{
	if ( !ForwardKinematic( state.joints, buffers.fk ) )
	{
		state.step = BacktrackStep( state.step );
		state.damping = BacktrackDamping( state.damping );
		state.fk_failures++;
		return;
	}


	PoseError( target, buffers.fk, buffers.error );
	const double current_error = buffers.error.squaredNorm();

	if ( current_error < state.error )
	{
		state.error = current_error;

		ComputeJacobianAndDamping( state.joints, state.damping, buffers );
		const double min_sv = GetMinSingularValue( buffers.jacobian );

		state.damping = ComputeAdaptiveDamping( min_sv );
		state.step = ComputeAdaptiveStep( min_sv );
		state.fk_failures = 0;
	}
	else
	{
		state.step = BacktrackStep( state.step );
		state.damping = BacktrackDamping( state.damping );
	}

	ComputeJacobianAndDamping( state.joints, state.damping, buffers );
	UpdateDeltaQ( buffers.jacobian, buffers.error, buffers.damped, buffers.dq );

	// std::cerr << "Joints = " << std::endl << state.joints << std::endl;
	// std::cerr << "dq = " << std::endl << buffers.dq << std::endl;
	// std::cerr << "Error = " << std::endl << buffers.error << std::endl;

	// state.joints.noalias() += state.step * buffers.dq;
	for ( int i = 0; i < state.joints.size(); i++ )
	{
		auto joint = state.joints[i];
		auto dq = buffers.dq[i];
		state.joints[i] = joint + dq;
	}
}

// ------------------------------------------------------------

void DLSKinematicsSolver::ComputeJacobianAndDamping(
	const VecXd& joints,
	double damping_factor,
	SolverBuffers& buffers ) const
{
	SpaceJacobian( twists_, joints, buffers.jacobian );

	const int n = buffers.jacobian.cols();
	buffers.damped.noalias() = buffers.jacobian.transpose() * buffers.jacobian;
	buffers.damped.diagonal().array() += damping_factor * damping_factor;
	// std::cerr << "Jac = " << std::endl << buffers.jacobian << std::endl;
	// std::cerr << "Damped = " << std::endl << buffers.damped << std::endl;
}

// ------------------------------------------------------------

void DLSKinematicsSolver::UpdateDeltaQ(
	const MatXd& jacobian,
	const VecXd& error,
	const MatXd& damped,
	VecXd& dq_out ) const
{
	buffers_.jac_transpose.noalias() = jacobian.transpose();
	buffers_.ldlt_solver.compute( damped );
	dq_out = buffers_.ldlt_solver.solve( buffers_.jac_transpose * error );

	// std::cerr << "jac_transpose = " << std::endl << buffers_.jac_transpose << std::endl;
	// std::cerr << "ldlt_solver = " << std::endl << buffers_.ldlt_solver.vectorD() << std::endl;
	// std::cerr << "jac_transpose * error = " << std::endl << buffers_.jac_transpose * error << std::endl;
	// std::cerr << "dq_out = " << std::endl << dq_out << std::endl;
}

// ------------------------------------------------------------

DLSKinematicsSolver::SolverState
DLSKinematicsSolver::EvaluateConvergence(
	const IterationState& state,
	int iteration ) const noexcept
{
	if ( state.error <= parameters_.error_tolerance )
	{
		return SolverState::Converged;
	}

	if ( iteration >= parameters_.max_iterations )
	{
		return SolverState::MaxIterations;
	}

	return SolverState::Improving;
}

// ------------------------------------------------------------

} // namespace SOArm100::Kinematics
