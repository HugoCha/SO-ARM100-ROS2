#pragma once

#include "Scorer/IKSolutionScorer.hpp"

#include <memory>
#include <vector>

namespace  SOArm100::Kinematics::Scorer
{
class WeightedScorer : public IKSolutionScorer
{
public:
using WeightScorerPair = std::pair< double, std::unique_ptr< const IKSolutionScorer >>;

WeightedScorer( std::vector< WeightScorerPair >&& scorers ) :
    scorers_( std::move( scorers ) )
{}

virtual double Score(
	const Solver::IKProblem& problem,
	const Solver::IKSolution& solution ) const override
{
    double score = 0.0;
    for ( const auto& ws : scorers_ )
        score += ws.first * ws.second->Score( problem, solution );
    return score;
}

private:
std::vector< WeightScorerPair > scorers_;
};
}