#include "DLSKinematicsSolver.hpp"

#include "Converter.hpp"
#include "KinematicsUtils.hpp"
#include "Types.hpp"

#include <Eigen/Dense>
#include <Eigen/src/SVD/JacobiSVD.h>
#include <vector>

namespace SOArm100::Kinematics
{

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
	const geometry_msgs::msg::Pose& target_pose )
{
	const Mat4d target = ToMat4d( target_pose );

	auto state = InitializeState( target );
	if ( !state )
	{
		state = InitializeState( target );
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
			state = InitializeState( target );
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
	std::vector< double >& joint_angles )
{
	auto result = SolveIK( target_pose );

	if ( result.Success() )
	{
		joint_angles = ToStdVector( result.joint_angles );
		return true;
	}

	return false;
}

// ------------------------------------------------------------

void DLSKinematicsSolver::PerformIteration(
	const Mat4d& target,
	IterationState& state,
	SolverBuffers& buffers )
{
	// Forward Kinematics
	if ( !ForwardKinematic( state.joints, buffers.fk ) )
	{
		state.step = BacktrackStep( state.step );
		state.damping = BacktrackDamping( state.damping );
		state.fk_failures++;
		return;
	}

	// Calculer l'erreur
	buffers.error = PoseError( target, buffers.fk );
	const double current_error = buffers.error.squaredNorm();

	// Vérifier amélioration
	if ( current_error < state.error )
	{
		// Amélioration → adapter les paramètres
		state.error = current_error;

		ComputeJacobianAndDamping( state.joints, state.damping, buffers );
		const double min_sv = GetMinSingularValue( buffers.jacobian );

		state.damping = ComputeAdaptiveDamping( min_sv );
		state.step = ComputeAdaptiveStep( min_sv );
		state.fk_failures = 0;
	}
	else
	{
		// Pas d'amélioration → backtrack
		state.step = BacktrackStep( state.step );
		state.damping = BacktrackDamping( state.damping );
	}

	// Calculer nouveau dq
	ComputeJacobianAndDamping( state.joints, state.damping, buffers );
	UpdateDeltaQ( buffers.jacobian, buffers.error, buffers.damped, buffers.dq );

	// Mise à jour
	state.joints.noalias() += state.step * buffers.dq;
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
