#pragma once

#include "Model/IKModelBase.hpp"
#include "Solver/IKSolverBase.hpp"

namespace SOArm100::Kinematics::Solver
{
struct IKProblem;
struct IKSolution;
class IKRunContext;

class HybridSolver : public Model::IKModelBase, public IKSolverBase
{
public:
HybridSolver( Model::KinematicModelConstPtr model );

virtual IKSolution Solve(
	const IKProblem& problem,
	const IKRunContext& context ) const override;
};
}
