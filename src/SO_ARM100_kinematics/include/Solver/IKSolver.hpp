#pragma once

#include "Heuristic/IKHeuristic.hpp"
#include "Heuristic/IKPresolution.hpp"
#include "IKSolution.hpp"

namespace SOArm100::Kinematics::Solver
{
struct IKProblem;
class IKRunContext;

class IKSolver : public Heuristic::IKHeuristic
{
public:
virtual ~IKSolver() = default;

IKSolver( Model::KinematicModelConstPtr model ) :
	Heuristic::IKHeuristic( model )
{
}

virtual IKSolution Solve(
	const IKProblem& problem,
	const IKRunContext& context ) const = 0;

virtual Heuristic::IKPresolution Presolve(
	const IKProblem& problem,
	const IKRunContext& context ) const override {
	return Solve( problem, context );
}
};
}