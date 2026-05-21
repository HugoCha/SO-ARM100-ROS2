#include "Solver/IKPipeline.hpp"

#include "Global.hpp"
#include "Seed/IKSeedGenerator.hpp"
#include "Solver/IKProblem.hpp"
#include "Solver/IKRunContext.hpp"

#include <memory>

namespace SOArm100::Kinematics::Solver
{

// ------------------------------------------------------------

IKPipeline::IKPipeline( 
    std::unique_ptr< const Seed::IKSeedGenerator > seed_generator,
    std::unique_ptr< const Heuristic::IIKHeuristic > heuristic,
    std::unique_ptr< const IIKSolver > solver ) :
    seed_generator_( std::move( seed_generator ) ),
    heuristic_( std::move( heuristic ) ),
    solver_( std::move( solver ) )
{}

// ------------------------------------------------------------

IKSolution IKPipeline::Run( 
    const IKProblem& problem, 
    const IKRunContext& context )
{
    auto pipeline_problem = problem;
    pipeline_problem.seed = seed_generator_->Generate( problem );
    
    auto presolution = heuristic_->Presolve( pipeline_problem, context );
    
    if ( presolution.PartialOrSuccess() )
        pipeline_problem.seed = presolution.joints;

    return solver_->Solve( pipeline_problem, context );
}

// ------------------------------------------------------------

}