#pragma once

#include "Heuristic/IIKHeuristic.hpp"
#include "Seed/IIKSeedGenerator.hpp"
#include "Solver/IIKSolver.hpp"

#include <memory>

namespace SOArm100::Kinematics::Solver
{
class IKPipeline;

class PipelineBuilder 
{
public:
PipelineBuilder& WithSeedGenerator( std::unique_ptr< const Seed::IIKSeedGenerator > gen );
PipelineBuilder& WithHeuristic( std::unique_ptr< const Heuristic::IIKHeuristic > h );
PipelineBuilder& WithSolver( std::unique_ptr< const IIKSolver > s );

std::unique_ptr< const IKPipeline > Build();

private:
std::unique_ptr< const Seed::IIKSeedGenerator > seed_gen_;
std::unique_ptr< const Heuristic::IIKHeuristic > heuristic_;
std::unique_ptr< const IIKSolver > solver_;
};
}