#include "DLS/DLSSolver.hpp"

#include "Global.hpp"

#include "DLS/AdaptativeDamping.hpp"
#include "DLS/DLSSolverState.hpp"
#include "Model/IKModelBase.hpp"
#include "Seed/IKRandomSeedGenerator.hpp"
#include "Solver/IKProblem.hpp"
#include "Solver/IKRunContext.hpp"
#include "Solver/IKSolution.hpp"
#include "Solver/IKSolverState.hpp"
#include "Solver/SolverType.hpp"
#include "Solver/SolverHistory.hpp"
#include "Utils/KinematicsUtils.hpp"

#include <Eigen/Dense>
#include <Eigen/src/Core/GlobalFunctions.h>
#include <Eigen/src/SVD/JacobiSVD.h>
#include <algorithm>
#include <cstddef>
#include <optional>
#include <ostream>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <string>

namespace SOArm100::Kinematics::Solver
{

// ------------------------------------------------------------
// Logger
// ------------------------------------------------------------

static rclcpp::Logger get_logger()
{
	return Logger::get().get_child( "DLSSolver" );
}

// ------------------------------------------------------------
// Solver Buffer
// ------------------------------------------------------------

struct DLSSolver::SolverBuffers {
	SolverType type;

	MatXd weights;
	MatXd jacobian;
	MatXd jacobian_psi;
	MatXd weighted_jacobian;
	Eigen::JacobiSVD< MatXd > svd;
	VecXd weighted_error;
	VecXd weighted_error_reachable;
	MatXd damped;
	VecXd gradient;
	VecXd dq;
	Mat4d fk;
	VecXd joints;
	Eigen::LDLT< MatXd > ldlt_solver;

	[[nodiscard]] inline int GetSize() const noexcept {
		return joints.size();
	}

	explicit SolverBuffers( int n_joints, SolverType solver_type ) :
		type( solver_type )
	{
		jacobian.resize( 6, n_joints );
		jacobian_psi.resize( n_joints, n_joints );
		damped.resize( n_joints, n_joints );
		joints.resize( n_joints );
		dq.resize( n_joints );
		gradient.resize( n_joints );

		if ( type == SolverType::Full )
		{
			weighted_jacobian.resize( 6, n_joints );
			weighted_error.resize( 6 );
			weighted_error_reachable.resize( 6 );
			weights.resize( 6, 6 );
		}
		else
		{
			weighted_jacobian.resize( 3, n_joints );
			weighted_error.resize( 3 );
			weighted_error_reachable.resize( 3 );
			weights.resize( 3, 6 );
		}
	}
};

// ------------------------------------------------------------
// Iteration State
// ------------------------------------------------------------

struct DLSSolver::IterationState {
	VecXd joints;
	double error;
	double error_reachable;
	double error_unreachable;
	double gradient;
	double step;
	double damping;
	int fk_failures;
	int stalled_error_iter;
};

// ------------------------------------------------------------
// DLS Solver
// ------------------------------------------------------------

DLSSolver::DLSSolver( Model::KinematicModelConstPtr model ) :
	DLSSolver( model, DefaultDLSSolverParameters() )
{
}

// ------------------------------------------------------------

DLSSolver::DLSSolver(
	Model::KinematicModelConstPtr model,
	DLSSolverParameters parameters ) :
	Model::IKModelBase( model ),
	parameters_( parameters )
{
	if ( !parameters_.IsValid() )
	{
		throw std::invalid_argument( "Invalid DLS configuration" );
	}
}

// ------------------------------------------------------------

IKSolution DLSSolver::Solve(
	const IKProblem& problem,
	const IKRunContext& context ) const
{
	if ( model_->IsUnreachable( problem.target ) )
	{
		return { IKSolverState::Unreachable, {}};
	}

	const int n_joints = GetChain()->GetActiveJointCount();

	if ( problem.seed.size() != n_joints )
	{
		RCLCPP_ERROR( get_logger(), "InitializeState: Joint vector size mismatch." );
		return { IKSolverState::NotRun, {}};
	}

	auto type = GetSolverType( parameters_ );
	SolverBuffers buffers = SolverBuffers{ n_joints, type };
	buffers.weights = InitializeWeightMatrix( parameters_.RotationWeightSqrt(), parameters_.TranslationWeightSqrt() );

	VecXd seed = GetChain()->ClampLimits( problem.seed );
	auto state = InitializeState( problem.target, seed, buffers );
	auto history = InitializeHistory( state, seed );
	auto seed_generator = InitializeSeedGenerator( state, seed );

	if ( !state )
	{
		if ( !problem.CanReSeed() )
			return { IKSolverState::NotRun, {}};

		seed = seed_generator.Generate( problem );
		state = InitializeState( problem.target, seed, buffers );
		history = InitializeHistory( state, seed );
		UpdateSeedGenerator( 0, *state, history, seed_generator );
	}

	for ( int iter = 0; iter < parameters_.max_iterations; ++iter )
	{
		// std::cout << "--------------- Iteration " << iter << " ---------------" << std::endl;
		if ( context.StopRequested() )
		{
			return { IKSolverState::NotRun, history.best_joints, history.best_error, iter };
		}

		auto solver_state = EvaluateConvergence( problem, *state, iter );

		if ( solver_state == DLSSolverState::Converged )
		{
			return { IKSolverState::Converged, state->joints, state->error, iter };
		}
		if ( solver_state == DLSSolverState::BestPossible )
		{
			return { IKSolverState::BestPossible, state->joints, state->error };
		}
		if ( solver_state == DLSSolverState::Stalled )
		{
			if ( !problem.CanReSeed() )
			{
				return { IKSolverState::BestPossible, history.best_joints, history.best_error, iter };
			}

			UpdateSeedGenerator( iter, *state, history, seed_generator );
			UpdateHistory( iter, *state, history );
			seed = seed_generator.Generate( problem );
			state = InitializeState( problem.target, seed, buffers );
			continue;
		}

		PerformIteration( problem, *state, buffers );
	}

	return { IKSolverState::MaxIterations, history.best_joints, history.best_error, parameters_.max_iterations };
}

// ------------------------------------------------------------

SolverType DLSSolver::GetSolverType( const DLSSolverParameters& parameters )
{
	if ( parameters.rotation_weight > 0 && parameters.translation_weight == 0 )
		return SolverType::Orientation;
	if ( parameters.rotation_weight == 0 && parameters.translation_weight > 0 )
		return SolverType::Position;
	return SolverType::Full;
}

// ------------------------------------------------------------

const MatXd DLSSolver::InitializeWeightMatrix(
	double rotation_weight,
	double translation_weight )
{
	MatXd weight_matrix;
	if ( rotation_weight > 0 && translation_weight == 0 )
	{
		weight_matrix = MatXd::Zero( 3, 6 );
		weight_matrix.block< 3, 3 >( 0, 0 ) = rotation_weight * Mat3d::Identity();
	}
	else if ( rotation_weight == 0 && translation_weight > 0 )
	{
		weight_matrix = MatXd::Zero( 3, 6 );
		weight_matrix.block< 3, 3 >( 0, 3 ) = translation_weight * Mat3d::Identity();
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

std::optional< DLSSolver::IterationState > DLSSolver::InitializeState(
	const Mat4d& target,
	const VecXd& initial_joints,
	SolverBuffers& buffers ) const
{
	IterationState state;
	state.joints = initial_joints;
	buffers.joints = initial_joints;

	if ( !UpdateBuffer( target, initial_joints, buffers ) )
	{
		RCLCPP_ERROR( get_logger(), "InitializeState: Buffer update failed." );
		return std::nullopt;
	}

	state.error = buffers.weighted_error.norm();
	state.error_reachable = buffers.weighted_error_reachable.norm();
	state.error_unreachable = UnreachableError( state.error, state.error_reachable );
	state.gradient = buffers.gradient.norm();

	state.stalled_error_iter = 0;
	state.step = parameters_.max_step;
	state.damping = parameters_.min_damping;
	state.fk_failures = 0;

	// std::cout << "Initial state" << std::endl;
	// PrintBuffer( buffers_ );
	// PrintState( state );

	return state;
}

// ------------------------------------------------------------

SolverHistory DLSSolver::InitializeHistory(
	const std::optional< IterationState >& state, const VecXd& initial_joints ) const
{
	SolverHistory history;

	if ( !state )
	{
		history.best_joints = initial_joints;
	}
	else
	{
		history.best_joints = state->joints;
		history.best_error = state->error;
	}

	return history;
}

// ------------------------------------------------------------

void DLSSolver::UpdateHistory(
	int iteration,
	const IterationState& state,
	SolverHistory& history ) const
{
	if ( state.error < history.best_error )
	{
		history.best_joints = state.joints;
		history.best_error = state.error;
		history.last_non_stalled_error_idx = history.restart_counter;
	}
	history.restart_counter++;
}

// ------------------------------------------------------------

Seed::IKRandomSeedGenerator DLSSolver::InitializeSeedGenerator(
	const std::optional< IterationState >& state, const VecXd& initial_joints ) const
{
	Seed::IKRandomSeedGenerator::RandomParameters random_parameters;
	auto random_type = Model::RandomType::Near;

	random_parameters.margin_percent = 0.1;
	if ( !state )
	{
		random_parameters.distance = 0.1;
	}
	else
	{
		double ratio = state->error_unreachable / state->error;
		random_parameters.distance = std::clamp( ratio, 0.1, 0.3 );
	}

	return Seed::IKRandomSeedGenerator{ model_, random_type, random_parameters };
}

// ------------------------------------------------------------

Model::RandomType DLSSolver::ChooseSeedRandomType(
	int iteration,
	const IterationState& state,
	const SolverHistory& history ) const
{
	if ( iteration == 0 &&
	     state.error_reachable / state.error < 0.5 &&
	     state.error >= 0.9 * history.best_error )
	{
		return Model::RandomType::NearCenterLimit;
	}

	int no_improvement_span = iteration - history.last_non_stalled_error_idx;
	if ( ( no_improvement_span <= 4 && history.best_error <= MediumError() ) ||
	     ( no_improvement_span <= 8 && history.best_error <= SmallError() ) )
	{
		return Model::RandomType::Near;
	}

	if ( ( state.error_reachable / state.error > 0.35 && state.error < 0.9 * history.best_error ) ||
	     state.error < 0.7 * history.best_error ||
	     ( no_improvement_span <= 3 && history.best_error <= MediumError() && state.error <= LargeError() ) )
	{
		return Model::RandomType::Near;
	}

	return Model::RandomType::Random;
}

// ------------------------------------------------------------

void DLSSolver::UpdateSeedGenerator(
	int iteration,
	const IterationState& state,
	const SolverHistory& history,
	Seed::IKRandomSeedGenerator& seed_generator ) const
{
	seed_generator.type = ChooseSeedRandomType( iteration, state, history );

	constexpr double min_distance = 0.05;
	constexpr double max_distance = 0.3;
	constexpr double avg_distance = ( min_distance + max_distance ) / 2.0;
	double distance = seed_generator.parameters.distance;

	if ( state.error <= SmallError() || history.best_error <= SmallError() )
	{
		if ( history.best_error > SmallError() ||
		     state.error < history.best_error )
			distance = min_distance;
		else
			distance = std::clamp( 1.15 * distance, min_distance, avg_distance );
	}
	else if ( state.error <= LargeError() )
	{
		if ( state.error < 0.7 * history.best_error )
			distance = ( min_distance + avg_distance ) / 2;
		else
			distance = avg_distance;
	}
	else
	{
		if ( state.error < 0.7 * history.best_error )
			distance = avg_distance;
		else
			distance = max_distance;
	}

	seed_generator.parameters.distance = std::clamp( distance, min_distance, max_distance );
}

// ------------------------------------------------------------

void DLSSolver::PerformIteration(
	const IKProblem& problem,
	IterationState& state,
	SolverBuffers& buffers ) const
{
	SolverBuffers state_buffers = buffers;

	Damped( buffers.weighted_jacobian, state.damping, buffers.damped );
	UpdateDeltaQ(
		buffers.ldlt_solver,
		buffers.damped,
		buffers.gradient,
		state.joints,
		buffers.weighted_jacobian,
		buffers.jacobian_psi,
		buffers.dq );
	LineSearch( problem.target, state, buffers );

	const double error = buffers.weighted_error_reachable.norm();
	UpdateErrorConvergence( problem, state.error_reachable, error, state );

	// PrintIteration( state, buffers );

	if ( state.error_reachable - error > problem.tolerance )
	{
		state.joints = buffers.joints;
		state.error = buffers.weighted_error.norm();
		state.error_reachable = buffers.weighted_error_reachable.norm();
		state.error_unreachable = UnreachableError( state.error, state.error_reachable );

		state.gradient = buffers.gradient.norm();

		state.fk_failures = 0;
		state.stalled_error_iter = 0;
		state.damping = AdaptativeDamping::LinearDamping(
			state.damping,
			parameters_.min_damping,
			parameters_.max_damping,
			0.5 );

		// PrintBuffer( buffers );
		// PrintState( state );
	}
	else
	{
		state.damping = AdaptativeDamping::LinearDamping(
			state.damping,
			parameters_.min_damping,
			parameters_.max_damping,
			4 );
		buffers = state_buffers;
	}
}

// ------------------------------------------------------------

void DLSSolver::LineSearch(
	const Mat4d& target,
	IterationState& state,
	SolverBuffers& buffers ) const
{
	assert( parameters_.line_search_factor< 1.0 && parameters_.line_search_factor >0.0 );
	double step = parameters_.max_step;

	do
	{
		buffers.joints = state.joints + step * buffers.dq;
		WrapJoints( buffers.joints );

		if ( !UpdateBuffer( target, buffers.joints, buffers ) )
		{
			state.fk_failures++;
			buffers.joints = state.joints;
			return;
		}
		if ( buffers.weighted_error_reachable.norm() >= state.error_reachable )
			step *= parameters_.line_search_factor;
	}
	while ( buffers.weighted_error_reachable.norm() >= state.error_reachable &&
	        step >= parameters_.min_step );

	state.step = step;
}

// ------------------------------------------------------------

void DLSSolver::WrapJoints( VecXd& joints ) const
{
	const auto& active_joints = GetChain()->GetActiveJoints();
	for ( size_t i = 0; i < active_joints.size(); i++ )
	{
		const auto joint = active_joints[i];
		const auto& limits = joint->GetLimits();
		if ( joint->IsRevolute() && limits.Span() >= 3 * M_PI / 2 )
		{
			if ( joints[i] > limits.Max() )
			{
				joints[i] = limits.Min() + ( joints[i] - limits.Max() );
			}
			else if ( joints[i] < limits.Min() )
			{
				joints[i] = limits.Max() - ( limits.Min() - joints[i] );
			}
		}
	}
}

// ------------------------------------------------------------

void DLSSolver::PrintIteration( const IterationState& state, const SolverBuffers& buffers ) const
{
	std::cout
	    << "step      = " << state.step << std::endl
	    << "damping   = " << state.damping << std::endl
	    << "Sjoints   = " << state.joints.transpose() << std::endl
	    << "dq        = " << buffers.dq.transpose() << std::endl
	    << "Bjoints   = " << buffers.joints.transpose() << std::endl
	    << "gradient  = " << buffers.gradient.transpose() << std::endl
	    << "Rerror     = " << buffers.weighted_error_reachable.transpose() << std::endl
	    << "Rerror norm= " << buffers.weighted_error_reachable.norm() << std::endl
	    << "error norm= " << buffers.weighted_error.norm() << std::endl;
}

// ------------------------------------------------------------

void DLSSolver::PrintState( const IterationState& state ) const
{
	std::cout
	    << "State" << std::endl
	    << "Werror  = " << state.error << std::endl
	    << "Rerror  = " << state.error_reachable << std::endl
	    << "Uerror  = " << state.error_unreachable << std::endl
	    << "Gradient= " << state.gradient << std::endl;
}

// ------------------------------------------------------------

void DLSSolver::PrintBuffer( const SolverBuffers& buffers ) const
{
	std::cout
	    << "Buffer" << std::endl
	    << "WJac  =" << std::endl << buffers.weighted_jacobian << std::endl
	    << "Joints= " << buffers.joints.transpose() << std::endl
	    << "Werror= " << buffers.weighted_error.transpose() << std::endl
	    << "Rerror= " << buffers.weighted_error_reachable.transpose() << std::endl;
}

// ------------------------------------------------------------

bool DLSSolver::UpdateBuffer(
	const Mat4d& target,
	const VecXd& joints,
	SolverBuffers& buffers ) const
{
	if ( !model_->ComputeFK( joints, buffers.fk ) )
		return false;

	SpaceJacobian( *GetChain(), joints, buffers.jacobian );
	WeightedPoseError( target, buffers.fk, parameters_.RotationWeightSqrt(), parameters_.TranslationWeightSqrt(), buffers.weighted_error );
	WeightedJacobian( buffers.jacobian, buffers.weights, buffers.weighted_jacobian );
	JacobianSVD( buffers.weighted_jacobian, buffers.svd );
	PseudoInverse( buffers.svd, parameters_.min_sv_tolerance, buffers.jacobian_psi );
	ReachableError( buffers.svd, buffers.weighted_error, parameters_.min_sv_tolerance, buffers.weighted_error_reachable );
	Gradient( buffers.weighted_jacobian, buffers.weighted_error_reachable, buffers.gradient );

	return true;
}

// ------------------------------------------------------------

void DLSSolver::UpdateErrorConvergence(
	const IKProblem& problem,
	double last_error,
	double current_error,
	IterationState& state ) const
{
	if ( last_error - current_error <= problem.tolerance )
	{
		state.stalled_error_iter++;
	}
	else
	{
		state.stalled_error_iter = 0;
	}
}

// ------------------------------------------------------------

void DLSSolver::UpdateDeltaQPrimary(
	Eigen::LDLT< MatXd >& ldlt_solver,
	const MatXd& damped,
	const VecXd& gradient,
	VecXd& dq ) const
{
	ldlt_solver.compute( damped );
	dq.noalias() = ldlt_solver.solve( gradient );
}

// ------------------------------------------------------------

void DLSSolver::UpdateDeltaQSecondary(
	const VecXd& joints,
	const MatXd& jacobian,
	const MatXd& jacobian_psi,
	VecXd& dq ) const
{
	MatXd N = MatXd::Identity( joints.size(), joints.size() ) - jacobian_psi * jacobian;
	VecXd centering{ joints.size() };
	const auto& active_joints = GetChain()->GetActiveJoints();
	for ( int i = 0; i < joints.size(); i++ )
	{
		const auto& joint_limit = active_joints[i]->GetLimits();
		centering[i] =  ( joints[i] - joint_limit.Center() ) / pow( joint_limit.Span(), 2 );
	}
	dq.noalias() = -0.005 * N * centering;
}

// ------------------------------------------------------------

void DLSSolver::ClampDeltaQ( VecXd& dq ) const
{
	if ( dq.norm() >= parameters_.max_dq )
		dq *= parameters_.max_dq / dq.norm();
}

// ------------------------------------------------------------

void DLSSolver::UpdateDeltaQ(
	Eigen::LDLT< MatXd >& ldlt_solver,
	const MatXd& damped,
	const VecXd& gradient,
	const VecXd& joints,
	const MatXd& jacobian,
	const MatXd& jacobian_psi,
	VecXd& dq ) const
{
	VecXd dq_primary = VecXd::Zero( joints.size() );
	VecXd dq_secondary = VecXd::Zero( joints.size() );
	UpdateDeltaQPrimary( ldlt_solver, damped, gradient, dq_primary );
	UpdateDeltaQSecondary( joints, jacobian, jacobian_psi, dq_secondary );

	dq.noalias() = dq_primary + dq_secondary;

	ClampDeltaQ( dq );
}

// ------------------------------------------------------------

DLSSolverState DLSSolver::EvaluateConvergence(
	const IKProblem& problem,
	const IterationState& state,
	int iteration ) const noexcept
{
	if ( state.error <= problem.tolerance )
	{
		return DLSSolverState::Converged;
	}

	if ( state.error_reachable < problem.tolerance &&
	     state.error_unreachable > problem.tolerance &&
	     state.gradient < parameters_.gradient_tolerance &&
	     state.stalled_error_iter > 0 )
	{
		return DLSSolverState::BestPossible;
	}

	if ( iteration >= parameters_.max_iterations )
	{
		return DLSSolverState::MaxIterations;
	}

	double unreachable_ratio = state.error_unreachable / state.error;
	double reachable_ratio = state.error_reachable / state.error;

	if ( ( state.gradient < parameters_.gradient_tolerance &&
	       state.error_reachable > problem.tolerance ) ||
	     ( unreachable_ratio  > 0.75 ) ||
	     ( unreachable_ratio  > 0.5 && reachable_ratio < 0.5 ) ||
	     ( state.stalled_error_iter > 0 && state.damping >= parameters_.max_damping && state.step <= parameters_.min_step ) ||
	     ( state.stalled_error_iter >= parameters_.max_stalle_iterations ) ||
	     ( state.error > LargeError() && state.stalled_error_iter > 0 ) ||
	     ( state.fk_failures >= 1 ) )
	{
		return DLSSolverState::Stalled;
	}

	return DLSSolverState::Improving;
}

// ------------------------------------------------------------

} // namespace SOArm100::Kinematics
