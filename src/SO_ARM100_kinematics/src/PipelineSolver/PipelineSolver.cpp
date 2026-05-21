#include "PipelineSolver/PipelineSolver.hpp"

#include "PipelineSolver/IKPipeline.hpp"
#include "Solver/IKProblem.hpp"
#include "Solver/IKRunContext.hpp"
#include "Solver/IKSolution.hpp"
#include "Solver/IKSolverState.hpp"

#include <condition_variable>

namespace SOArm100::Kinematics::Solver
{

// ------------------------------------------------------------

PipelineSolver::PipelineSolver(
	Model::KinematicModelConstPtr model,
	std::vector< std::unique_ptr< const Solver::IKPipeline >>&& pipelines,
	std::unique_ptr< Scorer::IKSolutionScorer >&& scorer,
	const SolverParameters& parameters ) :
	Model::IKModelBase( model ),
	pipelines_( std::move( pipelines ) ),
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

	auto worker =
		[&]( size_t index )
		{
			auto solution = RunAndScorePipeline(
				pipelines_[index],
				problem,
				context );

			if ( solution.state != IKSolverState::NotRun )
				solution.score = scorer_->Score( problem, solution );

			{
				std::lock_guard< std::mutex > lock( sync_params.mtx );
				if ( solution.score < result.score )
					result = solution;
			}

			if ( CanStopPipelines( solution ) )
			{
				sync_params.early_result = true;
				StopPipelines( context );
			}

			sync_params.completed_count++;
			sync_params.cv.notify_all();
		};

	auto pipeline_threads = StartPipelines( worker, problem, context );
	WaitPipelines( pipeline_threads, sync_params );

	return result;
}

// ------------------------------------------------------------

std::vector< std::thread > PipelineSolver::StartPipelines(
	auto worker,
	const IKProblem& problem,
	const IKRunContext& context ) const
{
	std::vector< std::thread > threads;

	for ( size_t i = 0; i < pipelines_.size(); ++i )
	{
		threads.emplace_back( worker, i );
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

IKSolution PipelineSolver::RunAndScorePipeline(
	const std::unique_ptr< const Solver::IKPipeline >& pipeline,
	const IKProblem& problem,
	const IKRunContext& context ) const
{
	auto solution = pipeline.get()->Run( problem, context );

	if ( solution.state != IKSolverState::NotRun )
		solution.score = scorer_->Score( problem, solution );

	return solution;
}

// ------------------------------------------------------------

void PipelineSolver::WaitPipelines(
	std::vector< std::thread >& pipeline_threads,
	SynchronizationParameters& sync_params ) const
{
	std::unique_lock< std::mutex > lock( sync_params.mtx );
	switch ( parameters_.strategy )
	{
	case PipelineCompletionStrategy::ReturnFirstSuccess:
	case PipelineCompletionStrategy::WaitForAcceptableResult:
		sync_params.cv.wait(
			lock,
			[&]{
				return sync_params.early_result ||
				       sync_params.completed_count == pipelines_.size();
			} );
		break;
	case PipelineCompletionStrategy::WaitForAllResults:
		sync_params.cv.wait(
			lock,
			[&]{
				return sync_params.completed_count == pipelines_.size();
			} );
		break;
	}

	for ( auto& t : pipeline_threads )
	{
		if ( t.joinable() )
			t.join();
	}
}

// ------------------------------------------------------------

}
