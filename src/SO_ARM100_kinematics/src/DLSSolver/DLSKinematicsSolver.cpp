#include "DLSSolver/DLSKinematicsSolver.hpp"

#include "DLSSolver/NumericSolverResult.hpp"
#include "DLSSolver/NumericSolverState.hpp"
#include "Global.hpp"
#include "KinematicsSolver.hpp"
#include "SolverType.hpp"
#include "Utils/Converter.hpp"
#include "Utils/KinematicsUtils.hpp"

#include <Eigen/Dense>
#include <Eigen/src/Core/GlobalFunctions.h>
#include <Eigen/src/SVD/JacobiSVD.h>
#include <cstddef>
#include <optional>
#include <ostream>
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
	: parameters_( parameters ),
	buffers_( 0, GetSolverType( parameters ) )
{
	if ( !parameters_.IsValid() )
	{
		throw std::invalid_argument( "Invalid DLS configuration" );
	}
}

// ------------------------------------------------------------

NumericSolverResult DLSKinematicsSolver::InverseKinematic(
	const Mat4d& target,
	const std::span< const double >& seed_joints ) const
{
	if ( KinematicsSolver::IsUnreachable( target ) )
	{
		return { NumericSolverState::Failed, {}, -1,  0 };
	}

	if ( seed_joints.size() != static_cast< int >( joint_chain_->GetActiveJointCount() ) )
	{
		RCLCPP_ERROR( get_logger(), "InitializeState: Joint vector size mismatch." );
		return { NumericSolverState::Failed, {}, -1, 0 };;
	}

	auto type = GetSolverType( parameters_ );
	if ( buffers_.GetSize() != static_cast< int >( joint_chain_->GetActiveJointCount() ) ||
	     buffers_.type != type )
	{
		buffers_ = SolverBuffers{ joint_chain_->GetActiveJointCount(), type };
		buffers_.weights = InitializeWeightMatrix( parameters_.rotation_weight, parameters_.translation_weight );
	}

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
		auto solver_state = EvaluateConvergence( *state, buffers_.jacobian.bottomRows( 3 ), buffers_.error.tail( 3 ), iter );
		if ( solver_state == NumericSolverState::Converged )
		{
			return { NumericSolverState::Converged, state->joints, state->error, iter };
		}

		if ( solver_state == NumericSolverState::Stalled )
		{
			state = InitializeState( target, RandomValidJointsNear( seed_joints, 0.03 ) );
			if ( !state )
			{
				return { NumericSolverState::Stalled, VecXd{}, 0.0, iter };
			}
			continue;
		}

		std::cout << "--------------- Iteration " << iter << " ---------------" << std::endl;

		PerformIteration( target, *state, buffers_ );

		// << "Damped=" << std::endl << buffers_.damped << std::endl;
	}

	return { NumericSolverState::MaxIterations, state->joints, state->error,
	         parameters_.max_iterations };
}

// ------------------------------------------------------------

bool DLSKinematicsSolver::InverseKinematicImpl(
	const Mat4d& target_pose,
	const std::span< const double >& seed_joints,
	double* joints ) const
{
	auto result = InverseKinematic( target_pose, seed_joints );

	if ( result.Success() )
	{
		std::copy( result.joints.begin(), result.joints.end(), joints );
		return true;
	}

	return false;
}

// ------------------------------------------------------------

SolverType DLSKinematicsSolver::GetSolverType( SolverParameters parameters )
{
	if ( parameters.rotation_weight > 0 && parameters.translation_weight == 0 )
		return SolverType::Orientation;
	if ( parameters.rotation_weight == 0 && parameters.translation_weight > 0 )
		return SolverType::Position;
	return SolverType::Full;
}

// ------------------------------------------------------------

const MatXd DLSKinematicsSolver::InitializeWeightMatrix(
	double rotation_weight,
	double translation_weight )
{
	MatXd weight_matrix;
	if ( rotation_weight > 0 && translation_weight == 0 )
	{
		weight_matrix = MatXd::Zero( 3, 6 );
		weight_matrix.diagonal().head( 3 ) = Vec3d::Constant( rotation_weight );
	}
	else if ( rotation_weight == 0 && translation_weight > 0 )
	{
		weight_matrix = MatXd::Zero( 3, 6 );
		weight_matrix.diagonal().tail( 3 ) = Vec3d::Constant( translation_weight );
	}
	else
	{
		weight_matrix = MatXd::Zero( 6, 6 );
		weight_matrix.diagonal().head( 3 ) = Vec3d::Constant( rotation_weight );
		weight_matrix.diagonal().tail( 3 ) = Vec3d::Constant( translation_weight );
	}
	return weight_matrix;
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

	SpaceJacobian( *joint_chain_, state.joints, buffers_.jacobian );
	WeightedJacobian( buffers_.jacobian, buffers_.weights, buffers_.weighted_jacobian );
	JacobianSVD( buffers_.weighted_jacobian, buffers_.svd );
	PoseError( target, buffers_.fk, buffers_.error );
	WeightedPoseError( buffers_.error, parameters_.rotation_weight, parameters_.translation_weight, buffers_.weighted_error );
	ReachableError( buffers_.svd, buffers_.weighted_error, parameters_.min_sv_tolerance, buffers_.weighted_error_reachable );
	Gradient( buffers_.weighted_jacobian, buffers_.weighted_error_reachable, buffers_.gradient );

	state.error = buffers_.error.squaredNorm();
	state.error_reachable = buffers_.weighted_error_reachable.squaredNorm();
	state.error_unreachable = ( buffers_.weighted_error - buffers_.weighted_error_reachable ).squaredNorm();
	state.gradient = buffers_.gradient.squaredNorm();

	state.stalled_error_iter = 0;
	state.step = parameters_.max_step;
	state.damping = parameters_.min_damping;
	state.fk_failures = 0;

	return state;
}

// ------------------------------------------------------------

const VecXd DLSKinematicsSolver::RandomValidJointsNear(
	const double* joints,
	double distance ) const noexcept
{
	VecXd random( joint_chain_->GetActiveJointCount() );
	random_numbers::RandomNumberGenerator rng;
	for ( size_t i = 0; i < joint_chain_->GetActiveJointCount(); i++ )
		joint_chain_->GetActiveJointLimits( i ).RandomNear( rng, joints[i], distance, & random.data()[i] );
	return random;
}

// ------------------------------------------------------------

const VecXd DLSKinematicsSolver::RandomValidJoints() const noexcept
{
	assert( joint_chain_ );
	VecXd random( joint_chain_->GetActiveJointCount() );
	random_numbers::RandomNumberGenerator rng;
	for ( size_t i = 0; i < joint_chain_->GetActiveJointCount(); i++ )
		joint_chain_->GetActiveJointLimits( i ).Random( rng, & random.data()[i] );
	return random;
}

// ------------------------------------------------------------

void DLSKinematicsSolver::PerformIteration(
	const Mat4d& target,
	IterationState& state,
	SolverBuffers& buffers ) const
{
	SpaceJacobian( *joint_chain_, state.joints, buffers.jacobian );
	WeightedJacobian( buffers.jacobian, buffers.weights, buffers.weighted_jacobian );
	Damped( buffers.weighted_jacobian, state.damping, buffers.damped );
	Gradient( buffers.weighted_jacobian, buffers.weighted_error, buffers.gradient );
	
	UpdateDeltaQ( buffers.damped, buffers.gradient, buffers.dq );
	buffers.joints.noalias() =  state.joints + state.step * buffers.dq;

	if ( !ForwardKinematic( buffers.joints, buffers.fk ) )
	{
		state.step = BacktrackStep( state.step );
		state.damping = BacktrackDamping( state.damping );
		state.fk_failures++;
		return;
	}

	JacobianSVD( buffers.weighted_jacobian, buffers.svd );
	PoseError( target, buffers.fk, buffers.error );
	WeightedPoseError( buffers.error, parameters_.rotation_weight, parameters_.translation_weight, buffers.weighted_error );
	ReachableError( buffers.svd, buffers.weighted_error, parameters_.min_sv_tolerance, buffers.weighted_error_reachable );
	const double current_error_reachable = buffers.weighted_error_reachable.squaredNorm();
	UpdateErrorConvergence( state.error_reachable, current_error_reachable, state );

	std::cout
	    << "damping=" << state.damping << std::endl
	    << "WJac=" << std::endl << buffers_.jacobian << std::endl
	    << "state joints=" << std::endl << state.joints.transpose() << std::endl
	    << "buffer joints=" << std::endl << buffers_.joints.transpose() << std::endl
	    << "step=" << state.step << std::endl
	    << "dq=" << std::endl << buffers_.dq.transpose() << std::endl
	    << "target position =" << Translation( target ).transpose() << std::endl
	    << "current position=" << Translation( buffers_.fk ).transpose() << std::endl
	    << "Buffer error="  << std::endl << buffers_.error.tail( 3 ).transpose() << std::endl
	    << "current error=" << current_error_reachable  << std::endl
	    << "state error=" << state.error  << std::endl;

	if ( current_error_reachable < state.error_reachable )
	{
		state.error_reachable = current_error_reachable;
		state.error = buffers.weighted_error.squaredNorm();
		state.error_unreachable = ( buffers.weighted_error - buffers.weighted_error_reachable ).squaredNorm();
		state.gradient = buffers.gradient.squaredNorm();

		state.joints.noalias() = buffers.joints;
		state.fk_failures = 0;

		if ( state.min_sv <= parameters_.min_sv_tolerance )
		{
			state.damping = ComputeAdaptiveDamping( state.min_sv );
			state.step = ComputeAdaptiveStep( state.min_sv );
		}

		std::cout << "min_sv=" << state.min_sv << std::endl;
	}
	else
	{
		state.damping = BacktrackDamping( state.damping );
		state.step = BacktrackStep( state.step );
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

void DLSKinematicsSolver::UpdateDeltaQ(
	const MatXd& damped,
	const VecXd& gradient,
	VecXd& dq ) const
{
	buffers_.ldlt_solver.compute( damped );
	dq.noalias() = buffers_.ldlt_solver.solve( gradient );

	if ( dq.norm() > parameters_.max_dq )
		dq *= ( parameters_.max_dq ) / dq.norm();
}

// ------------------------------------------------------------

NumericSolverState DLSKinematicsSolver::EvaluateConvergence(
	const IterationState& state,
	const MatXd& jacobian,
	const VecXd& error,
	int iteration ) const noexcept
{
	if ( state.error <= parameters_.error_tolerance )
	{
		return NumericSolverState::Converged;
	}

	if ( state.error_reachable < parameters_.error_tolerance &&
	     state.error_unreachable > parameters_.error_tolerance )
	{
		return NumericSolverState::BestPossible;
	}

	if ( iteration >= parameters_.max_iterations )
	{
		return NumericSolverState::MaxIterations;
	}

	if ( ( state.gradient < parameters_.gradient_tolerance &&
	       state.error_reachable > parameters_.error_tolerance ) ||
	     ( state.stalled_error_iter >= parameters_.max_stalle_iterations ) ||
	     ( state.fk_failures >= parameters_.max_stalle_iterations ) )
	{
		return NumericSolverState::Stalled;
	}

	return NumericSolverState::Improving;
}

// ------------------------------------------------------------

} // namespace SOArm100::Kinematics
