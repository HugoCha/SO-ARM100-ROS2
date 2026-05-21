#pragma once

#include "Seed/IKSeedGenerator.hpp"
#include "Heuristic/IIKHeuristic.hpp"
#include "IIKSolver.hpp"

#include <memory>

namespace SOArm100::Kinematics::Solver
{
class IKPipeline
{
IKPipeline( std::unique_ptr< const Seed::IKSeedGenerator > seed_generator,
            std::unique_ptr< const Heuristic::IIKHeuristic > heuristic,
            std::unique_ptr< const IIKSolver > solver );

IKSolution Run( 
    const IKProblem& problem, 
    const IKRunContext& context );

private:
std::unique_ptr< const Seed::IKSeedGenerator > seed_generator_;
std::unique_ptr< const Heuristic::IIKHeuristic > heuristic_;
std::unique_ptr< const IIKSolver > solver_;
};
}