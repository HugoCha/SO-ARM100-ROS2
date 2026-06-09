#pragma once

#include "Scorer/IKSolutionScorer.hpp"

namespace SOArm100::Kinematics::Scorer
{
class SeedConsistencyScorer : public IKSolutionScorer
{
public:
SeedConsistencyScorer( double consistency_penalty );

virtual double Score(
	const Solver::IKProblem& problem,
	const Solver::IKSolution& solution ) const override;

private:
double consistency_penalty_;
};
}