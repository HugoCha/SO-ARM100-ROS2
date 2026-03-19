#pragma once

#include "Solver/IKSolver.hpp"

namespace SOArm100::Kinematics::Solver
{
struct IKProblem;
struct IKSolution;
class IKRunContext;

class HybridSolver : public IKSolver
{
public:
HybridSolver( Model::KinematicModelConstPtr model );

virtual IKSolution Solve(
	const IKProblem& problem,
	const IKRunContext& context ) const override;
};
}
