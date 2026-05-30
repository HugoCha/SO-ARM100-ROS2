#include "Heuristic/TopologyHeuristic.hpp"

#include "Heuristic/IIKHeuristic.hpp"
#include "Heuristic/IKPresolution.hpp"
#include "Model/Joint/JointGroup.hpp"
#include "Solver/IKProblem.hpp"

namespace SOArm100::Kinematics::Heuristic
{

// ------------------------------------------------------------

TopologyHeuristic::TopologyHeuristic( Model::KinematicModelConstPtr model ) :
    Model::IKModelBase( model )
{
    InitializeHeuristic( model );
}

// ------------------------------------------------------------

IKPresolution TopologyHeuristic::Presolve(
	const Solver::IKProblem& problem,
	const Solver::IKRunContext& context ) const
{
    auto intermediate_problem = problem;
    Heuristic::IKPresolution presolution;

    auto topology = model_->GetTopology();

    if ( topology.Get( Model::revolute_base_name ) )
    {
        presolution = base_heuristic_->Presolve( problem, context );
        if ( presolution.PartialOrSuccess() )
            intermediate_problem.seed = presolution.joints;
    }

    presolution = intermediate_heuristic_->Presolve( intermediate_problem, context );
    if ( presolution.PartialOrSuccess() )
        intermediate_problem.seed = presolution.joints;

    if ( topology.Get( Model::wrist_name ) )
    {
        presolution = base_heuristic_->Presolve( problem, context );
    }

    return presolution;
}

// ------------------------------------------------------------

}