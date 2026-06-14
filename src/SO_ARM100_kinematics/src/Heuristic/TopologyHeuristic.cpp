#include "Heuristic/TopologyHeuristic.hpp"

#include "FABRIK/FabrikSolver.hpp"
#include "Global.hpp"

#include "Heuristic/IIKHeuristic.hpp"
#include "Heuristic/IKHeuristicState.hpp"
#include "Heuristic/IKPresolution.hpp"
#include "Heuristic/IKPresolutionBranch.hpp"
#include "Heuristic/PlanarNRHeuristic.hpp"
#include "Heuristic/PrismaticBaseHeuristic.hpp"
#include "Heuristic/RevoluteBaseHeuristic.hpp"
#include "Heuristic/WristHeuristic.hpp"
#include "Model/Joint/JointGroup.hpp"
#include "Solver/IKProblem.hpp"
#include "Solver/IKRunContext.hpp"

#include <memory>

namespace SOArm100::Kinematics::Heuristic
{

// ------------------------------------------------------------

TopologyHeuristic::TopologyHeuristic( Model::KinematicModelConstPtr model ) :
	Model::IKModelBase( model ),
	base_heuristic_( nullptr ),
	planar_heuristic_( nullptr ),
	wrist_heuristic_( nullptr ),
	fabrik_heuristic_( nullptr )
{

	auto topology = model->GetTopology();

	if ( topology.Get( Model::revolute_base_name ) )
	{
		base_heuristic_ = std::make_unique< Heuristic::RevoluteBaseHeuristic >(
			model,
			*topology.Get( Model::revolute_base_name ) );
	}
	else if ( topology.Get( Model::prismatic_base_name ) )
	{
		base_heuristic_ = std::make_unique< Heuristic::PrismaticBaseHeuristic >(
			model,
			*model->GetTopology().Get( Model::prismatic_base_name ) );
	}

	if ( topology.Get( Model::planarNR_name ) )
	{
		planar_heuristic_ = std::make_unique< PlanarNRHeuristic >(
			model,
			*topology.Get( Model::planarNR_name ) );
	}

	if ( topology.Get( Model::fallback_fabrik ) )
	{
		fabrik_heuristic_ = std::make_unique< Solver::FABRIKSolver >( model );
	}

	if ( topology.Get( Model::wrist_name ) )
	{
		wrist_heuristic_ = std::make_unique< Heuristic::WristHeuristic >(
			model,
			*topology.Get( Model::wrist_name ) );
	}

}

// ------------------------------------------------------------

IKPresolution TopologyHeuristic::Presolve(
	const Solver::IKProblem& problem,
	const Solver::IKRunContext& context ) const
{
	if ( model_->IsUnreachable( problem.target ) )
		return { {}, IKHeuristicState::Fail };

	double problem_error = model_->ComputeError( problem.seed, problem.target );
	if ( problem_error < problem.tolerance )
		return { {{problem.seed,problem_error,0}}, IKHeuristicState::Success };

	auto topology = model_->GetTopology();

	auto expand_heuristic = [&]( const IIKHeuristic* heuristic, 
										   const IKPresolution& previous_presolution,
										   IKPresolution& next_presolution ) -> bool
							 {
								next_presolution.branches.clear();

								for ( const auto& branch : previous_presolution.branches )
								{
									auto branch_problem = problem;
									branch_problem.seed = branch.joints;
									auto branch_presolution = heuristic->Presolve( branch_problem, context );
									if ( branch_presolution.PartialOrSuccess() )
									{
										next_presolution.branches.insert( 
											next_presolution.branches.end(),
											branch_presolution.branches.begin(), 
											branch_presolution.branches.end() );
									}
								}

								return !next_presolution.branches.empty();
							 };

	auto previous_presolution = IKPresolution { {{problem.seed} } };
	auto next_presolution = previous_presolution;
	if ( base_heuristic_ && !expand_heuristic( base_heuristic_.get(), previous_presolution, next_presolution ) )
	{
		return { {}, IKHeuristicState::Fail };
	}

	previous_presolution = next_presolution;
	if ( planar_heuristic_ && !expand_heuristic( planar_heuristic_.get(), previous_presolution, next_presolution ) )
	{
		return { {}, IKHeuristicState::Fail };
	}

	previous_presolution = next_presolution;
	if ( fabrik_heuristic_ && !expand_heuristic( fabrik_heuristic_.get(), previous_presolution, next_presolution ) )
	{
		return { {}, IKHeuristicState::Fail };
	}

	previous_presolution = next_presolution;
	if ( wrist_heuristic_ && !expand_heuristic( wrist_heuristic_.get(), previous_presolution, next_presolution ) )
	{
		return { {}, IKHeuristicState::Fail };
	}

	for ( auto& branch : next_presolution.branches )
		branch.error = model_->ComputeError( branch.joints, problem.target );

	return next_presolution;
}

// ------------------------------------------------------------

}