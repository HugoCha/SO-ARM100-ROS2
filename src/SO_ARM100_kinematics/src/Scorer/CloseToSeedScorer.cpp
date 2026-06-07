#include "Scorer/CloseToSeedScorer.hpp"

#include "Global.hpp"
#include "Scorer/IKSolutionScorer.hpp"
#include "Solver/IKProblem.hpp"
#include "Solver/IKSolution.hpp"
#include "Utils/Distance.hpp"

namespace SOArm100::Kinematics::Scorer
{

// ------------------------------------------------------------

CloseToSeedScorer::CloseToSeedScorer( Model::KinematicModelConstPtr model ) :
	model_( model ),
	max_distance_( ComputeMaxDistance( model ) )
{
}

// ------------------------------------------------------------

double CloseToSeedScorer::ComputeMaxDistance( Model::KinematicModelConstPtr model )
{
	const auto& joints = model->GetChain()->GetActiveJoints();
	const int n_joints = joints.size();

	VecXd max_distance( n_joints );

	for ( int i = 0; i < n_joints; i++ )
	{
		const auto& limit = joints[i]->GetLimits();
		max_distance[i] = limit.Span() / 2.0;
	}

	return max_distance.norm();
}

// ------------------------------------------------------------

double CloseToSeedScorer::Score(
	const Solver::IKProblem& problem,
	const Solver::IKSolution& solution ) const
{
	return Utils::Distance(
		problem.seed,
		solution.joints ) / max_distance_;
}

// ------------------------------------------------------------

}