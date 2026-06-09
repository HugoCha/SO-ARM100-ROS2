#pragma once

#include "Heuristic/IIKHeuristic.hpp"
#include "Seed/IIKSeedGenerator.hpp"
#include "Solver/IIKSolver.hpp"

#include <memory>

namespace SOArm100::Kinematics::Solver
{
class IKPipeline : public IIKSolver
{
public:
IKPipeline( std::unique_ptr< const Seed::IIKSeedGenerator > seed_generator,
            std::unique_ptr< const Heuristic::IIKHeuristic > heuristic,
            std::unique_ptr< const IIKSolver > solver );

virtual IKSolution Solve(
	const IKProblem& problem,
	const IKRunContext& context ) const override;

private:
std::unique_ptr< const Seed::IIKSeedGenerator > seed_generator_;
std::unique_ptr< const Heuristic::IIKHeuristic > heuristic_;
std::unique_ptr< const IIKSolver > solver_;
};
}