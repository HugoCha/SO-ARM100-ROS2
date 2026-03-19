#include "HybridSolver/HybridSolver.hpp"

#include "Solver/IKProblem.hpp"
#include "Solver/IKRunContext.hpp"
#include "Solver/IKSolver.hpp"
#include "Solver/IKSolution.hpp"

namespace SOArm100::Kinematics::Solver
{

// ------------------------------------------------------------

HybridSolver::HybridSolver( Model::KinematicModelConstPtr model ) :
	IKSolver( model )
{
}

// ------------------------------------------------------------

IKSolution HybridSolver::Solve(
	const IKProblem& problem,
	const IKRunContext& context ) const
{
	return {};
}

// ------------------------------------------------------------

}
