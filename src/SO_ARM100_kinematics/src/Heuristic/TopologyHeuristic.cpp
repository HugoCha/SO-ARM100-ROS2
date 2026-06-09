#include "Heuristic/TopologyHeuristic.hpp"

#include "FABRIK/FabrikSolver.hpp"
#include "Global.hpp"

#include "Heuristic/IIKHeuristic.hpp"
#include "Heuristic/IKHeuristicState.hpp"
#include "Heuristic/IKPresolution.hpp"
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

	if ( topology.Get( Model::wrist_name ) )
	{
		wrist_heuristic_ = std::make_unique< Heuristic::WristHeuristic >(
			model,
			*topology.Get( Model::wrist_name ) );
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
}

// ------------------------------------------------------------

IKPresolution TopologyHeuristic::Presolve(
	const Solver::IKProblem& problem,
	const Solver::IKRunContext& context ) const
{
	if ( model_->IsUnreachable( problem.target ) )
		return { problem.seed, IKHeuristicState::Fail }
	;

	auto intermediate_problem = problem;
	Heuristic::IKPresolution global_presolution;
	global_presolution.state = IKHeuristicState::Success;

	auto topology = model_->GetTopology();

	auto compute_heuristic = [&]( const IIKHeuristic* heuristic ) -> bool
							 {
								 auto local_presolution = heuristic->Presolve( intermediate_problem, context );
								 global_presolution.error += local_presolution.error;
								 global_presolution.iterations += local_presolution.iterations;
								 intermediate_problem.seed = local_presolution.joints;

								 if ( local_presolution.PartialOrSuccess() )
								 {
									 if ( local_presolution.state == IKHeuristicState::PartialSuccess )
										 global_presolution.state = IKHeuristicState::PartialSuccess;
									 return true;
								 }
								 return false;
							 };

	if ( base_heuristic_ && !compute_heuristic( base_heuristic_.get() ) )
	{
		return {
		    intermediate_problem.seed,
		    IKHeuristicState::Fail,
		    global_presolution.error,
		    global_presolution.iterations };
	}

	if ( planar_heuristic_ && !compute_heuristic( planar_heuristic_.get() ) )
	{
		return {
		    intermediate_problem.seed,
		    IKHeuristicState::Fail,
		    global_presolution.error,
		    global_presolution.iterations };
	}

	if ( fabrik_heuristic_ && !compute_heuristic( fabrik_heuristic_.get() ) )
	{
		return {
		    intermediate_problem.seed,
		    IKHeuristicState::Fail,
		    global_presolution.error,
		    global_presolution.iterations };
	}

	if ( wrist_heuristic_ && !compute_heuristic( wrist_heuristic_.get() ) )
	{
		return {
		    intermediate_problem.seed,
		    IKHeuristicState::Fail,
		    global_presolution.error,
		    global_presolution.iterations };
	}

	global_presolution.error = model_->ComputeError( intermediate_problem.seed, problem.target );
	global_presolution.joints = intermediate_problem.seed;
	return global_presolution;
}

// ------------------------------------------------------------

}