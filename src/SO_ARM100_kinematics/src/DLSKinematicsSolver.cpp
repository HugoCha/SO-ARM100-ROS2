#include "DLSKinematicsSolver.hpp"

#include "Converter.hpp"
#include "KinematicsUtils.hpp"

#include <Eigen/Dense>
#include <Eigen/src/SVD/JacobiSVD.h>
#include <cstddef>
#include <moveit/robot_model/joint_model.hpp>
#include <moveit/robot_model/link_model.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <optional>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <string>

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

NumericSolverResult DLSKinematicsSolver::SolveIK(
	const Mat4d& target,
	const std::span< const double >& seed_joints ) const
{
	if ( seed_joints.size() != static_cast< int >( twists_.size() ) )
	{
		RCLCPP_ERROR( get_logger(), "InitializeState: Joint vector size mismatch." );
		return { NumericSolverState::Failed, {}, -1, 0 };;
	}

	if ( buffers_.GetSize() != static_cast< int >( twists_.size() ) )
	{
		buffers_ = SolverBuffers{ twists_.size() };
	}

	const auto& weights = InitializeWeightMatrix();
	auto state = InitializeState( target, ToVecXd( seed_joints ) );
	if ( !state )
	{
		state = InitializeState( target, RandomValidJoints() );
		if ( !state )
		{
			return { NumericSolverState::Failed, VecXd{}, 0.0, 0 };
		}
	}

	for ( int iter = 0; iter < parameters_.max_iterations; ++iter )
	{
		auto solver_state = EvaluateConvergence( *state, iter );
		if ( solver_state == NumericSolverState::Converged )
		{
			return { NumericSolverState::Converged, state->joints, state->error, iter };
		}

		if ( IsStalled( *state ) )
		{
			state = InitializeState( target, RandomValidJoints() );
			if ( !state )
			{
				return { NumericSolverState::Stalled, VecXd{}, 0.0, iter };
			}
			continue;
		}

		PerformIteration( target, weights, *state, buffers_ );
	}

	return { NumericSolverState::MaxIterations, state->joints, state->error,
	         parameters_.max_iterations };
}

// ------------------------------------------------------------

bool DLSKinematicsSolver::InverseKinematic(
	const Mat4d& target_pose,
	const std::span< const double >& seed_joints,
	VecXd& joint_angles ) const
{
	auto result = SolveIK( target_pose, seed_joints );

	if ( result.Success() )
	{
		joint_angles = result.joint_angles;
		return true;
	}

	return false;
}

// ------------------------------------------------------------

const Mat6d DLSKinematicsSolver::InitializeWeightMatrix() const
{
	Eigen::VectorXd weights( 6 );
	weights << parameters_.rotation_weight, parameters_.rotation_weight, parameters_.rotation_weight,
	    parameters_.translation_weight, parameters_.translation_weight, parameters_.translation_weight;
	return weights.asDiagonal();
}

// ------------------------------------------------------------

std::optional< DLSKinematicsSolver::IterationState > DLSKinematicsSolver::InitializeState(
	const Mat4d& target,
	const VecXd& initial_joints ) const
{
	IterationState state;
	state.joints = initial_joints;
	buffers_.joints = initial_joints;

	if ( !ForwardKinematic( state.joints, buffers_.fk ) )
	{
		RCLCPP_ERROR( get_logger(), "InitializeState: Forward Kinematics failed." );
		return std::nullopt;
	}

	WeightedPoseError(
		target,
		buffers_.fk,
		parameters_.rotation_weight,
		parameters_.translation_weight,
		buffers_.error );

	state.stalled_error_iter = 0;
	state.error = buffers_.error.squaredNorm();
	state.step = parameters_.max_step;
	state.damping = parameters_.min_damping;
	state.fk_failures = 0;

	return state;
}

// ------------------------------------------------------------

const VecXd DLSKinematicsSolver::RandomValidJoints() const noexcept
{
	assert( p_joint_model_group_ );
	VecXd random( joint_models_.size() );
	random_numbers::RandomNumberGenerator rng;
	for ( size_t i = 0; i < joint_models_.size(); i++ )
		joint_models_[i]->getVariableRandomPositions( rng, &random.data()[i] );
	return random;
}

// ------------------------------------------------------------

void DLSKinematicsSolver::PerformIteration(
	const Mat4d& target,
	const Mat6d& weights,
	IterationState& state,
	SolverBuffers& buffers ) const
{
	ComputeWeightedJacobianAndDamping( state.joints, weights, state.damping, buffers );
	UpdateDeltaQ( buffers.jacobian, buffers.error, buffers.damped, buffers.dq );
	buffers.joints.noalias() =  state.joints + state.step * buffers.dq;

	if ( !ForwardKinematic( buffers.joints, buffers.fk ) )
	{
		state.step = BacktrackStep( state.step );
		state.damping = BacktrackDamping( state.damping );
		state.fk_failures++;
		return;
	}

	WeightedPoseError(
		target,
		buffers.fk,
		parameters_.rotation_weight,
		parameters_.translation_weight,
		buffers.error );
	const double current_error = buffers.error.squaredNorm();
	UpdateErrorConvergence( state.error, current_error, state );

	if ( current_error < state.error )
	{
		state.error = current_error;

		ComputeWeightedJacobianAndDamping( state.joints, weights, state.damping, buffers );
		const double min_sv = GetMinSingularValue( buffers.jacobian );

		state.damping = ComputeAdaptiveDamping( min_sv );
		state.step = ComputeAdaptiveStep( min_sv );
		state.joints.noalias() = buffers.joints;
		state.fk_failures = 0;
	}
	else
	{
		state.step = BacktrackStep( state.step );
		state.damping = BacktrackDamping( state.damping );
	}
}

// ------------------------------------------------------------

void DLSKinematicsSolver::UpdateErrorConvergence(
	double last_error,
	double current_error,
	IterationState& state ) const
{
	if ( abs( last_error - current_error ) <= parameters_.error_tolerance )
	{
		state.stalled_error_iter++;
	}
	else
	{
		state.stalled_error_iter = 0;
	}
}

// ------------------------------------------------------------

void DLSKinematicsSolver::ComputeWeightedJacobianAndDamping(
	const VecXd& joints,
	const Mat6d& weights,
	double damping_factor,
	SolverBuffers& buffers ) const
{
	SpaceJacobian( twists_, joints, buffers.jacobian );
	buffers.jacobian.noalias() = weights * buffers.jacobian;

	const int n = buffers.jacobian.cols();
	buffers.damped.noalias() = buffers.jacobian.transpose() * buffers.jacobian;
	buffers.damped.diagonal().array() += damping_factor * damping_factor;
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
}

// ------------------------------------------------------------

NumericSolverState DLSKinematicsSolver::EvaluateConvergence(
	const IterationState& state,
	int iteration ) const noexcept
{
	if ( state.error <= parameters_.error_tolerance )
	{
		return NumericSolverState::Converged;
	}

	if ( iteration >= parameters_.max_iterations )
	{
		return NumericSolverState::MaxIterations;
	}

	return NumericSolverState::Improving;
}

// ------------------------------------------------------------

} // namespace SOArm100::Kinematics
