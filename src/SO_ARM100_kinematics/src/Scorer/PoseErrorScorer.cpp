#include "Scorer/PoseErrorScorer.hpp"

#include "Model/KinematicModel.hpp"
#include "Solver/IKSolution.hpp"
#include "Solver/IKProblem.hpp"

namespace SOArm100::Kinematics::Scorer
{

// ------------------------------------------------------------

PoseErrorScorer::PoseErrorScorer( 
    Model::KinematicModelConstPtr model, 
    ScorerParameters parameters ) :
    model_( model ),
    parameters_( parameters )
{}

// ------------------------------------------------------------

double PoseErrorScorer::Score(
	const Solver::IKProblem& problem,
	const Solver::IKSolution& solution ) const
{
    return solution.error > parameters_.error_tolerance ? 
        parameters_.violation_penalty * solution.error : 0.0;
}

// ------------------------------------------------------------

}