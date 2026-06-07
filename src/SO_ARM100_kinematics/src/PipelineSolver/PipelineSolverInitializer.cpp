#include "PipelineSolver/PipelineSolverInitializer.hpp"

#include "DLS/DLSSolver.hpp"
#include "Heuristic/TopologyHeuristic.hpp"
#include "Model/Joint/JointGroup.hpp"
#include "Model/KinematicModel.hpp"
#include "PipelineSolver/IKPipeline.hpp"
#include "PipelineSolver/PipelineSolver.hpp"
#include "PipelineSolver/PipelineSolverParameters.hpp"
#include "PipelineSolver/PipelineBuilder.hpp"
#include "Scorer/CloseToCenterScorer.hpp"
#include "Scorer/CloseToSeedScorer.hpp"
#include "Scorer/ManipulabilityScorer.hpp"
#include "Scorer/PoseErrorScorer.hpp"
#include "Scorer/WeightedScorersBuilder.hpp"
#include "Seed/IKOppositeSeedGenerator.hpp"

namespace SOArm100::Kinematics::Solver
{

std::unique_ptr< Solver::PipelineSolver > PipelineSolverInitializer::Initialize(
	Model::KinematicModelConstPtr model,
	const PipelineSolverParameters& parameters )
{
	std::vector< std::unique_ptr< const Solver::IKPipeline >> pipelines;

	Solver::DLSSolver::SolverParameters params_safe_solver;
	params_safe_solver.max_iterations = 500;
	params_safe_solver.max_stalle_iterations = 2;
	Solver::DLSSolver::SolverParameters params_fast_solver;
	params_fast_solver.max_iterations = 250;
	params_fast_solver.max_stalle_iterations = 2;

	pipelines.emplace_back(
		Solver::PipelineBuilder{}
		.WithHeuristic( std::make_unique< Heuristic::TopologyHeuristic >( model ) )
		.WithSolver( std::make_unique< Solver::DLSSolver >( model, params_fast_solver ) )
		.Build() );

	if ( model->GetTopology().Get( Model::revolute_base_name ) )
	{
		auto revolute_base_group = *model->GetTopology().Get( Model::revolute_base_name );
		pipelines.emplace_back(
			Solver::PipelineBuilder{}
			.WithSeedGenerator( std::make_unique< Seed::IKOppositeSeedGenerator >( model, revolute_base_group ) )
			.WithHeuristic( std::make_unique< Heuristic::TopologyHeuristic >( model ) )
			.WithSolver( std::make_unique< Solver::DLSSolver >( model, params_fast_solver ) )
			.Build() );
	}

	if ( model->GetTopology().Get( Model::planarNR_name ) )
	{
		auto planar_group = *model->GetTopology().Get( Model::planarNR_name );
		pipelines.emplace_back(
			Solver::PipelineBuilder{}
			.WithSeedGenerator( std::make_unique< Seed::IKOppositeSeedGenerator >( model, planar_group ) )
			.WithHeuristic( std::make_unique< Heuristic::TopologyHeuristic >( model ) )
			.WithSolver( std::make_unique< Solver::DLSSolver >( model, params_fast_solver ) )
			.Build() );
	}

	pipelines.emplace_back(
		Solver::PipelineBuilder{}
		.WithSeedGenerator( std::make_unique< Seed::IKOppositeSeedGenerator >( model ) )
		.WithHeuristic( std::make_unique< Heuristic::TopologyHeuristic >( model ) )
		.WithSolver( std::make_unique< Solver::DLSSolver >( model, params_fast_solver ) )
		.Build() );

	pipelines.emplace_back(
		Solver::PipelineBuilder{}
		.WithSolver( std::make_unique< Solver::DLSSolver >( model, params_safe_solver ) )
		.Build() );

	auto scorer = Scorer::WeightedScorersBuilder{}
	.Add( 1.0, std::make_unique< Scorer::CloseToCenterScorer >( model ) )
	.Add( 3.0, std::make_unique< Scorer::CloseToSeedScorer >( model ) )
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