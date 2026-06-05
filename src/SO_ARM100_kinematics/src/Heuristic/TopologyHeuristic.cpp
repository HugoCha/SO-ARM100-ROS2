#include "Heuristic/TopologyHeuristic.hpp"

#include "FABRIK/FabrikSolver.hpp"
#include "Global.hpp"

#include "Heuristic/IIKHeuristic.hpp"
#include "Heuristic/IKHeuristicState.hpp"
#include "Heuristic/IKPresolution.hpp"
#include "Heuristic/PlanarNRHeuristic.hpp"
#include "Heuristic/RevoluteBaseHeuristic.hpp"
#include "Heuristic/WristHeuristic.hpp"
#include "Model/Joint/JointGroup.hpp"
#include "Solver/IKProblem.hpp"
#include "Solver/IKRunContext.hpp"

#include <limits>
#include <memory>

namespace SOArm100::Kinematics::Heuristic
{

// ------------------------------------------------------------

TopologyHeuristic::TopologyHeuristic( Model::KinematicModelConstPtr model ) :
    Model::IKModelBase( model ),
    base_heuristic_( nullptr ),
    intermediate_heuristic_( nullptr ),
    wrist_heuristic_( nullptr ) 
{

    auto topology = model->GetTopology();

    if ( topology.Get( Model::revolute_base_name ) )
    {
        base_heuristic_ = std::make_unique< Heuristic::RevoluteBaseHeuristic >( 
            model, 
            *topology.Get( Model::revolute_base_name ) );
    }

    if ( topology.Get( Model::wrist_name ) )
    {
        wrist_heuristic_ = std::make_unique< Heuristic::WristHeuristic >( 
            model, 
            *topology.Get( Model::wrist_name ) );
    }

    if ( topology.Get( Model::planarNR_name ) )
    {
        intermediate_heuristic_ = std::make_unique< PlanarNRHeuristic >( 
            model,
            *topology.Get( Model::planarNR_name )
        );
    }
    else 
    {
        intermediate_heuristic_ = std::make_unique< Solver::FABRIKSolver >( model );
    }
}

// ------------------------------------------------------------

IKPresolution TopologyHeuristic::Presolve(
	const Solver::IKProblem& problem,
	const Solver::IKRunContext& context ) const
{
    auto intermediate_problem = problem;
    Heuristic::IKPresolution presolution;
    Heuristic::IKPresolution global_presolution;
    global_presolution.state = IKHeuristicState::Success;

    auto topology = model_->GetTopology();

    auto compute_heuristic = [&]( const IIKHeuristic* heuristic ) -> bool
    {
        auto local_presolution = heuristic->Presolve( intermediate_problem, context );
        if ( local_presolution.PartialOrSuccess() )
        {
            intermediate_problem.seed = local_presolution.joints;
            global_presolution.error += local_presolution.error;
            global_presolution.iterations += local_presolution.iterations;
            if ( local_presolution.state == IKHeuristicState::PartialSuccess )
                global_presolution.state = IKHeuristicState::PartialSuccess;
            return true;
        }
        return false;
    };

    if ( topology.Get( Model::revolute_base_name ) && 
         !compute_heuristic( base_heuristic_.get() ) )
    {
        return { 
            intermediate_problem.seed, 
            IKHeuristicState::Fail, 
            std::numeric_limits<double>::infinity(), 
            presolution.iterations };
    }

    if ( !compute_heuristic( intermediate_heuristic_.get() ) )
    {
        return { 
            intermediate_problem.seed, 
            IKHeuristicState::Fail, 
            std::numeric_limits<double>::infinity(), 
            presolution.iterations };
    }

    if ( topology.Get( Model::wrist_name ) &&
         !compute_heuristic( wrist_heuristic_.get() ) )
    {
        return { 
            intermediate_problem.seed, 
            IKHeuristicState::Fail, 
            std::numeric_limits<double>::infinity(), 
            presolution.iterations };
    }

    global_presolution.joints = intermediate_problem.seed;
    return global_presolution;
}

// ------------------------------------------------------------

}