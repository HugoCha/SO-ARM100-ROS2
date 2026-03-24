#pragma once

namespace SOArm100::Kinematics
{
namespace Solver
{
class IKProblem;
class IKRunContext;
}

namespace Heuristic
{
struct IKPresolution;

class IIKHeuristic
{
public:
virtual ~IIKHeuristic() = default;

virtual IKPresolution Presolve(
	const Solver::IKProblem& problem,
	const Solver::IKRunContext& context ) const = 0;
};
}
}