#pragma once

#include "Heuristic/IIKHeuristic.hpp"
#include "Solver/IIKSolver.hpp"

namespace SOArm100::Kinematics::Solver
{
class IKSolverBase : public IIKSolver, public Heuristic::IIKHeuristic
{
public:
virtual ~IKSolverBase() = default;

virtual Heuristic::IKPresolution Presolve(
	const IKProblem& problem,
	const IKRunContext& context ) const override {
	return Solve( problem, context );
}
};
}