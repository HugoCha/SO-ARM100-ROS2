#include "Scorer/SeedConsistencyScorer.hpp"

#include "Global.hpp"

#include "Scorer/IKSolutionScorer.hpp"
#include "Solver/IKProblem.hpp"
#include "Solver/IKSolution.hpp"

namespace SOArm100::Kinematics::Scorer
{

// ------------------------------------------------------------

SeedConsistencyScorer::SeedConsistencyScorer( double consistency_penalty ) :
	consistency_penalty_( consistency_penalty )
{
}

// ------------------------------------------------------------

double SeedConsistencyScorer::Score(
	const Solver::IKProblem& problem,
	const Solver::IKSolution& solution ) const
{
	if ( problem.consistency.size() != solution.joints.size() )
		return 0.0;

	double score = 0.0;

	for ( int i = 0; i < solution.joints.size(); i++ )
	{
		double distance = std::abs( problem.seed[i] - solution.joints[i] );
		if ( distance > std::abs( problem.consistency[i] ) )
			score += consistency_penalty_ * distance;
	}

	return score;
}

// ------------------------------------------------------------

}