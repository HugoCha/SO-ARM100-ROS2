#pragma once

namespace SOArm100::Kinematics
{
namespace Solver
{
struct IKProblem;
struct IKSolution;
}
namespace Scorer
{
class IKSolutionScorer
{
public:
virtual ~IKSolutionScorer() = default;

virtual double Score(
	const Solver::IKProblem& problem,
	const Solver::IKSolution& solution ) const = 0;
};
}
}