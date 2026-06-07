#pragma once

#include "IKPipeline.hpp"
#include "Model/IKModelBase.hpp"
#include "PipelineSolverParameters.hpp"
#include "Scorer/IKSolutionScorer.hpp"
#include "Solver/IKSolverBase.hpp"

#include <condition_variable>
#include <memory>
#include <thread>

namespace SOArm100::Kinematics::Solver
{
struct IKProblem;
struct IKSolution;
class IKRunContext;

class PipelineSolver : public Model::IKModelBase, public IKSolverBase
{
public:
PipelineSolver(
	Model::KinematicModelConstPtr model,
	std::vector< std::unique_ptr< const Solver::IKPipeline >>&& pipelines,
	std::unique_ptr< Scorer::IKSolutionScorer >&& scorer,
	const PipelineSolverParameters& parameters );

PipelineSolverParameters& Parameters(){
	return parameters_;
}

const PipelineSolverParameters& Parameters() const {
	return parameters_;
}

virtual IKSolution Solve(
	const IKProblem& problem,
	const IKRunContext& context ) const override;

private:
struct SynchronizationParameters
{
	std::mutex mtx;
	std::condition_variable cv;
	int completed_count = 0;
	bool early_result = false;
};

std::vector< std::unique_ptr< const Solver::IKPipeline >> pipelines_;
std::unique_ptr< Scorer::IKSolutionScorer > scorer_;
PipelineSolverParameters parameters_;

std::vector< std::thread > StartPipelines(
	auto worker,
	const IKProblem& problem,
	const IKRunContext& context ) const;

IKSolution RunAndScorePipeline(
	const std::unique_ptr< const Solver::IKPipeline >& pipeline,
	const IKProblem& problem,
	const IKRunContext& context ) const;

bool CanStopPipelines( const IKSolution& solution ) const;

void WaitPipelines(
	std::vector< std::thread >& pipeline_threads,
	const IKProblem& problem,
	const IKRunContext& context,
	SynchronizationParameters& sync_params ) const;

void StopPipelines( const IKRunContext& context ) const;
};
}
