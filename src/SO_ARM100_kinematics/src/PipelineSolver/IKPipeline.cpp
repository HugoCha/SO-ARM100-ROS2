#include "PipelineSolver/IKPipeline.hpp"

#include "Global.hpp"
#include "Heuristic/IKHeuristicState.hpp"
#include "Heuristic/IKPresolution.hpp"
#include "Seed/IIKSeedGenerator.hpp"
#include "Solver/IKProblem.hpp"
#include "Solver/IKRunContext.hpp"
#include "Solver/IKSolution.hpp"
#include "Solver/IKSolverState.hpp"
#include "Utils/StringConverter.hpp"

#include <memory>

namespace SOArm100::Kinematics::Solver
{

// ------------------------------------------------------------

IKPipeline::IKPipeline(
	std::unique_ptr< const Seed::IIKSeedGenerator > seed_generator,
	std::unique_ptr< const Heuristic::IIKHeuristic > heuristic,
	std::unique_ptr< const IIKSolver > solver ) :
	seed_generator_( std::move( seed_generator ) ),
	heuristic_( std::move( heuristic ) ),
	solver_( std::move( solver ) )
{
}

// ------------------------------------------------------------

Heuristic::IKPresolution IKPipeline::Presolve(
	const IKProblem& problem,
	const IKRunContext& context ) const
{
	auto heuristic_problem = problem;
	auto presolution = Heuristic::IKPresolution{{{ problem.seed }}, Heuristic::IKHeuristicState::PartialSuccess };

	if ( seed_generator_ )
		heuristic_problem.seed = seed_generator_->Generate( problem );

	if ( heuristic_ )
		presolution = heuristic_->Presolve( heuristic_problem, context );

	return presolution;
}

// ------------------------------------------------------------

IKSolution IKPipeline::Solve(
	const IKProblem& problem,
	const IKRunContext& context ) const
{
	IKSolution solution = { IKSolverState::NotRun, {}};

	if ( solver_ )
		solution = solver_->Solve( problem, context );

	return solution;
}

// ------------------------------------------------------------

}