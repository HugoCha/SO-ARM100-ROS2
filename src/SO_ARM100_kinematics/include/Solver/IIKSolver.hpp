#pragma once

#include "IKSolution.hpp"

namespace SOArm100::Kinematics::Solver
{
struct IKProblem;
class IKRunContext;

class IIKSolver
{
public:
virtual ~IIKSolver() = default;

virtual IKSolution Solve(
	const IKProblem& problem,
	const IKRunContext& context ) const = 0;
};
}