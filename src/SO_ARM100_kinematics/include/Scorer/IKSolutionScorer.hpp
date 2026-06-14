#pragma once

#include "Global.hpp"
#include "Solver/IKSolution.hpp"

namespace SOArm100::Kinematics
{
namespace Solver
{
struct IKProblem;
}
namespace Scorer
{
class IKSolutionScorer
{
public:
virtual ~IKSolutionScorer() = default;

virtual double Score(
	const Solver::IKProblem& problem,
	const VecXd& solution,
	double error ) const = 0;

double Score(
	const Solver::IKProblem& problem,
	const Solver::IKSolution& solution ) const {
	return Score( problem, solution.joints, solution.error );
}
};
}
}