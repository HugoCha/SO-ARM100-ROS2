#pragma once

#include "Model/IKModelBase.hpp"
#include "Solver/IKPipeline.hpp"
#include "Solver/IKSolverBase.hpp"

namespace SOArm100::Kinematics::Solver
{
struct IKProblem;
struct IKSolution;
class IKRunContext;

class PipelineSolver : public Model::IKModelBase, public IKSolverBase
{
public:
enum class PipelineHandle
{
First,
WaitAcceptable,
WaitAll,
};

PipelineSolver( 
	Model::KinematicModelConstPtr model,
	std::vector< Solver::IKPipeline > pipelines );

virtual IKSolution Solve(
	const IKProblem& problem,
	const IKRunContext& context ) const override;

private:
std::vector< Solver::IKPipeline > pipelines_;

};
}
