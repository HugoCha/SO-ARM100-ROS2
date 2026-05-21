#include "PipelineSolver/IKPipeline.hpp"

#include "Global.hpp"
#include "Seed/IIKSeedGenerator.hpp"
#include "Solver/IKProblem.hpp"
#include "Solver/IKRunContext.hpp"
#include "Solver/IKSolution.hpp"
#include "Utils/StringConverter.hpp"

#include <exception>
#include <memory>
#include <iostream>

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
{}

// ------------------------------------------------------------

IKSolution IKPipeline::Run( 
    const IKProblem& problem, 
    const IKRunContext& context ) const
{
    auto pipeline_problem = problem;
    auto pipeline_solution = IKSolution{};

    std::cout << "Initial" << std::endl;
    std::cout << problem << std::endl;
    if ( seed_generator_ )
        pipeline_problem.seed = seed_generator_->Generate( problem );

    std::cout << "Seed" << std::endl;
    std::cout << pipeline_problem << std::endl;

    if ( heuristic_ )
    {
        auto presolution = heuristic_->Presolve( pipeline_problem, context );
        if ( presolution.PartialOrSuccess() )
            pipeline_problem.seed = presolution.joints;
        std::cout << "Heuristic" << std::endl;
        std::cout << presolution << std::endl;
    }
    
    pipeline_solution.joints = pipeline_problem.seed;

    if ( solver_ )
        pipeline_solution = solver_->Solve( pipeline_problem, context );

    std::cout << "Solver" << std::endl;
    std::cout << pipeline_solution << std::endl;

    return pipeline_solution;
}

// ------------------------------------------------------------

}