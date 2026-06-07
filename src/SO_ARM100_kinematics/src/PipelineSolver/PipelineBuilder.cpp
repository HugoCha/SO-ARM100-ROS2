#include "PipelineSolver/PipelineBuilder.hpp"

#include "PipelineSolver/IKPipeline.hpp"

#include <memory>

namespace SOArm100::Kinematics::Solver
{

// ------------------------------------------------------------

PipelineBuilder& PipelineBuilder::WithSeedGenerator(
	std::unique_ptr< const Seed::IIKSeedGenerator > gen )
{
	seed_gen_ = std::move( gen );
	return *this;
}

// ------------------------------------------------------------

PipelineBuilder& PipelineBuilder::WithHeuristic(
	std::unique_ptr< const Heuristic::IIKHeuristic > h )
{
	heuristic_ = std::move( h );
	return *this;
}

// ------------------------------------------------------------

PipelineBuilder& PipelineBuilder::WithSolver(
	std::unique_ptr< const IIKSolver > s )
{
	solver_ = std::move( s );
	return *this;
}

// ------------------------------------------------------------

std::unique_ptr< const IKPipeline > PipelineBuilder::Build()
{
	return std::make_unique< const IKPipeline >(
		std::move( seed_gen_ ),
		std::move( heuristic_ ),
		std::move( solver_ ) );
}

// ------------------------------------------------------------

}