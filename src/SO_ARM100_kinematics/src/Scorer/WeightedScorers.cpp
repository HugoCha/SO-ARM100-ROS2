#include "Scorer/WeightedScorers.hpp"

namespace  SOArm100::Kinematics::Scorer
{

// ------------------------------------------------------------

WeightedScorers::WeightedScorers( std::vector< WeightScorerPair >&& scorers ) :
    scorers_( std::move( scorers ) )
{}

// ------------------------------------------------------------

double WeightedScorers::Score(
	const Solver::IKProblem& problem,
	const Solver::IKSolution& solution ) const
{
    double score = 0.0;
    double w_sum = 0.0;
    for ( const auto& ws : scorers_ )
    {
        score += ws.first * ws.second->Score( problem, solution );
        w_sum += ws.first;
    }
    return score / w_sum;
}

// ------------------------------------------------------------

}