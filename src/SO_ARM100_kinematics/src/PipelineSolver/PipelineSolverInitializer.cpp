#include "PipelineSolver/PipelineSolverInitializer.hpp"

#include "DLS/DLSSolver.hpp"
#include "DLS/DLSSolverParameters.hpp"
#include "Heuristic/TopologyHeuristic.hpp"
#include "Model/KinematicModel.hpp"
#include "PipelineSolver/IKPipeline.hpp"
#include "PipelineSolver/PipelineSolver.hpp"
#include "PipelineSolver/PipelineSolverParameters.hpp"
#include "PipelineSolver/PipelineBuilder.hpp"
#include "Scorer/CloseToCenterScorer.hpp"
#include "Scorer/CloseToSeedScorer.hpp"
#include "Scorer/ManipulabilityScorer.hpp"
#include "Scorer/PoseErrorScorer.hpp"
#include "Scorer/SeedConsistencyScorer.hpp"
#include "Scorer/WeightedScorersBuilder.hpp"
#include "Seed/IKOppositeSeedGenerator.hpp"

#include <limits>

namespace SOArm100::Kinematics::Solver
{

// ------------------------------------------------------------

std::unique_ptr< const Solver::IKPipeline > PipelineSolverInitializer::InitializeSinglePipeline(
	Model::KinematicModelConstPtr model )
{
	return Solver::PipelineBuilder{}
		.WithHeuristic( std::make_unique< Heuristic::TopologyHeuristic >( model ) )
		.WithSolver( std::make_unique< Solver::DLSSolver >( model, RobustDLSSolverParameters() ) )
		.Build();
}

// ------------------------------------------------------------

std::unique_ptr< const Solver::PipelineSolver > PipelineSolverInitializer::InitializeMultiplePipeline(
	Model::KinematicModelConstPtr model,
	const PipelineSolverParameters& parameters )
{
	std::vector< std::unique_ptr< const Solver::IKPipeline >> pipelines;

	pipelines.emplace_back(
		Solver::PipelineBuilder{}
		.WithHeuristic( std::make_unique< Heuristic::TopologyHeuristic >( model ) )
		.WithSolver( std::make_unique< Solver::DLSSolver >( model, DefaultDLSSolverParameters() ) )
		.Build() );

	pipelines.emplace_back(
		Solver::PipelineBuilder{}
		.WithSeedGenerator( std::make_unique< Seed::IKOppositeSeedGenerator >( model ) )
		.WithHeuristic( std::make_unique< Heuristic::TopologyHeuristic >( model ) )
		.WithSolver( std::make_unique< Solver::DLSSolver >( model, FastDLSSolverParameters() ) )
		.Build() );

	auto scorer = Scorer::WeightedScorersBuilder{}
	.Add( 1.0, std::make_unique< Scorer::CloseToCenterScorer >( model ) )
	.Add( 3.0, std::make_unique< Scorer::CloseToSeedScorer >( model ) )
	.Add( 1.0, std::make_unique< Scorer::SeedConsistencyScorer >( std::numeric_limits<double>::infinity() ) )
	.Add( 3.0, std::make_unique< Scorer::ManipulabilityScorer >( model ) )
	.Add( 1.0, std::make_unique< Scorer::PoseErrorScorer >( model, Scorer::PoseErrorScorer::ScorerParameters() ) )
	.Build();

	return std::unique_ptr< Solver::PipelineSolver >( new Solver::PipelineSolver(
														  model,
														  std::move( pipelines ),
														  std::move( scorer ),
														  parameters ) );
}

}