#include "PipelineSolver/PipelineSolver.hpp"

#include "Solver/IKProblem.hpp"
#include "Solver/IKRunContext.hpp"
#include "Solver/IKSolution.hpp"

namespace SOArm100::Kinematics::Solver
{

// ------------------------------------------------------------

PipelineSolver::PipelineSolver( 
	Model::KinematicModelConstPtr model,
	std::vector< Solver::IKPipeline > pipelines ) :
	Model::IKModelBase( model ),
	pipelines_( pipelines )
{
}

// ------------------------------------------------------------

IKSolution PipelineSolver::Solve(
	const IKProblem& problem,
	const IKRunContext& context ) const
{
	return {};
}

// ------------------------------------------------------------

}
