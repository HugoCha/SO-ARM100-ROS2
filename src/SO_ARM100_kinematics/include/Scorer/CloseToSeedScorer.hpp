#pragma once

#include "Scorer/IKSolutionScorer.hpp"
#include "Solver/IKProblem.hpp"
#include "Solver/IKSolution.hpp"
#include "Utils/Distance.hpp"

namespace SOArm100::Kinematics::Scorer
{
class CloseToSeedScorer : public IKSolutionScorer
{
public:
CloseToSeedScorer(){}

virtual double Score(
	const Solver::IKProblem& problem,
	const Solver::IKSolution& solution ) const override
{
    return Utils::Distance( 
        problem.seed, 
        solution.joints );
}
};
}