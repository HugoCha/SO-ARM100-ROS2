#pragma once

#include "Seed/IKSeedGenerator.hpp"
#include "Heuristic/IKHeuristic.hpp"
#include "IKSolver.hpp"

#include <memory>

namespace SOArm100::Kinematics::Solver
{
class IKPipeline
{
IKPipeline( const Seed::IKSeedGenerator& seed_generator,
            const Heuristic::IKHeuristic& heuristic,
            const IKSolver& solver );

IKSolution Run( IKProblem problem, IKRunContext context );

private:
std::unique_ptr< const Seed::IKSeedGenerator > seed_generator_;
std::unique_ptr< const Heuristic::IKHeuristic > heuristic_;
std::unique_ptr< const IKSolver > solver_;
};
}