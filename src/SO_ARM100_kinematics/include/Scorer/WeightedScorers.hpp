#pragma once

#include "Scorer/IKSolutionScorer.hpp"

#include <memory>
#include <vector>

namespace  SOArm100::Kinematics::Scorer
{
class WeightedScorers : public IKSolutionScorer
{
public:
using WeightScorerPair = std::pair< double, std::unique_ptr< const IKSolutionScorer >>;

WeightedScorers( std::vector< WeightScorerPair >&& scorers );

virtual double Score(
	const Solver::IKProblem& problem,
	const Solver::IKSolution& solution ) const override;

private:
std::vector< WeightScorerPair > scorers_;
};
}