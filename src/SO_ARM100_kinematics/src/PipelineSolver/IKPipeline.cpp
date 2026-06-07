#include "PipelineSolver/IKPipeline.hpp"

#include "Global.hpp"
#include "Seed/IIKSeedGenerator.hpp"
#include "Solver/IKProblem.hpp"
#include "Solver/IKRunContext.hpp"
#include "Solver/IKSolution.hpp"
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

IKSolution IKPipeline::Run(
	const IKProblem& problem,
	const IKRunContext& context ) const
{
	auto pipeline_problem = problem;
	auto pipeline_solution = IKSolution{};
	int heuristic_iterations = 0;

	if ( seed_generator_ )
		pipeline_problem.seed = seed_generator_->Generate( problem );

	if ( heuristic_ )
	{
		auto presolution = heuristic_->Presolve( pipeline_problem, context );
		heuristic_iterations = presolution.iterations;
		if ( presolution.PartialOrSuccess() )
			pipeline_problem.seed = presolution.joints;
	}

	pipeline_solution.joints = pipeline_problem.seed;

	if ( solver_ )
	{
		pipeline_solution = solver_->Solve( pipeline_problem, context );
		pipeline_solution.iterations += heuristic_iterations;
	}

	return pipeline_solution;
}

// ------------------------------------------------------------

}