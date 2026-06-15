#include "PipelineSolver/PipelineSolver.hpp"

#include "PipelineSolver/IKPipeline.hpp"
#include "Solver/IKProblem.hpp"
#include "Solver/IKRunContext.hpp"
#include "Solver/IKSolution.hpp"
#include "Solver/IKSolverState.hpp"
#include "Utils/StringConverter.hpp"

#include <condition_variable>
#include <limits>

namespace SOArm100::Kinematics::Solver
{

// ------------------------------------------------------------

PipelineSolver::PipelineSolver(
	Model::KinematicModelConstPtr model,
	std::unique_ptr< const Solver::IKPipeline >&& pipeline,
	std::unique_ptr< Scorer::IKSolutionScorer >&& scorer,
	const PipelineSolverParameters& parameters ) :
	Model::IKModelBase( model ),
	pipeline_( std::move( pipeline ) ),
	scorer_( std::move( scorer ) ),
	parameters_( parameters )
{
}

// ------------------------------------------------------------

IKSolution PipelineSolver::Solve(
	const IKProblem& problem,
	const IKRunContext& context ) const
{
	IKSolution result = {};

	SynchronizationParameters sync_params;

	auto presolution = pipeline_->Presolve( problem, context );

	for ( auto& branch : presolution.branches )
		branch.cost = scorer_->Score( problem, branch.joints, branch.error );

	std::sort( presolution.branches.begin(), presolution.branches.end(),
	           []( const Heuristic::IKPresolutionBranch& b1, const Heuristic::IKPresolutionBranch& b2 ){
			return b1.cost < b2.cost;
		} );

	// IKSolution best_solution {};
	// for ( const auto& branch : presolution.branches )
	// {
	// 	auto branch_solution = RunAndScoreBranch( branch, problem, context );
	// 	if ( branch_solution.score < best_solution.score )
	// 	{
	// 		best_solution = branch_solution;
	// 	}
	// 	if ( branch_solution.Success() )
	// 	{
	// 		return branch_solution;
	// 	}
	// }
	if ( presolution.branches.size() > 1 )
	{
		auto worker =
			[&]( const IKProblem& problem,
			     const IKRunContext& context,
			     const Heuristic::IKPresolutionBranch& branch,
			     SynchronizationParameters& sync_parameters )
			{
				auto solution = RunAndScoreBranch(
					branch,
					problem,
					context );

				{
					std::lock_guard< std::mutex > lock( sync_parameters.mtx );
					if ( solution.score < result.score )
						result = solution;

					if ( CanStopPipelines( solution ) )
					{
						StopPipelines( context );
						sync_parameters.early_result = true;
					}

					sync_parameters.completed_count++;
				}
				sync_parameters.cv.notify_all();
			};

		auto pipeline_threads = StartPipelines( worker, sync_params, presolution, problem, context );
		WaitPipelines( pipeline_threads, presolution, problem, context, sync_params );
	}
	else if ( presolution.branches.size() == 1 )
	{
		result = RunAndScoreBranch( presolution.branches[0], problem, context );
	}

	return result;
}

// ------------------------------------------------------------

std::vector< std::thread > PipelineSolver::StartPipelines(
	auto worker,
	SynchronizationParameters& sync_params,
	const Heuristic::IKPresolution& presolution,
	const IKProblem& problem,
	const IKRunContext& context ) const
{
	std::vector< std::thread > threads;
	uint max_parallel_threads = std::max( 1u, parameters_.max_parallel_thread );
	max_parallel_threads = std::min( ( uint )presolution.branches.size(), max_parallel_threads );
	std::atomic< size_t > next_branch{ 0 };

	for ( auto w = 0; w < max_parallel_threads; ++w )
	{
		threads.emplace_back( [&]
			{
				while ( true )
				{
					if ( context.StopRequested() )
						return;

					size_t idx = next_branch.fetch_add( 1 );

					if ( idx >= presolution.branches.size() )
						return;

					worker( problem, context, presolution.branches[idx], sync_params );
				}
			} );
	}
	return threads;
}

// ------------------------------------------------------------

bool PipelineSolver::CanStopPipelines( const IKSolution& solution ) const
{
	bool can_stop =
		parameters_.strategy == PipelineCompletionStrategy::ReturnFirstSuccess;

	can_stop |=
		parameters_.strategy == PipelineCompletionStrategy::WaitForAcceptableResult &&
		solution.score <= parameters_.min_score_threshold;

	can_stop &= solution.Success();

	return can_stop;
}

// ------------------------------------------------------------

void PipelineSolver::StopPipelines( const IKRunContext& context ) const
{
	context.RequestStop();
}

// ------------------------------------------------------------

IKSolution PipelineSolver::RunAndScoreBranch(
	const Heuristic::IKPresolutionBranch& branch,
	const IKProblem& problem,
	const IKRunContext& context ) const
{
	if ( std::isinf( branch.cost ) )
		return { IKSolverState::NotRun, {}}
	;

	auto branch_problem = problem;
	branch_problem.seed = branch.joints;
	auto solution = pipeline_->Solve( branch_problem, context );

	if ( solution.state != IKSolverState::NotRun &&
	     solution.state != IKSolverState::Unreachable )
		solution.score = scorer_->Score( problem, solution );

	return solution;
}

// ------------------------------------------------------------

void PipelineSolver::WaitPipelines(
	std::vector< std::thread >& pipeline_threads,
	const Heuristic::IKPresolution& presolution,
	const IKProblem& problem,
	const IKRunContext& context,
	SynchronizationParameters& sync_params ) const
{
	std::unique_lock< std::mutex > lock( sync_params.mtx );
	auto timeout { std::chrono::milliseconds( problem.timeout_ms ) };
	bool completed = true;
	switch ( parameters_.strategy )
	{
	case PipelineCompletionStrategy::ReturnFirstSuccess:
	case PipelineCompletionStrategy::WaitForAcceptableResult:
		completed = sync_params.cv.wait_for(
			lock,
			timeout,
			[&]{
				return sync_params.early_result ||
				       sync_params.completed_count == presolution.branches.size();
			} );
		break;
	case PipelineCompletionStrategy::WaitForAllResults:
		completed = sync_params.cv.wait_for(
			lock,
			timeout,
			[&]{
				return sync_params.completed_count == presolution.branches.size();
			} );
		break;
	}

	if ( !completed )
		StopPipelines( context );
	lock.unlock();

	for ( auto& t : pipeline_threads )
	{
		if ( t.joinable() )
			t.join();
	}
}

// ------------------------------------------------------------

}
