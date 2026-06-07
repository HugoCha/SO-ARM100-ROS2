#pragma once

namespace SOArm100::Kinematics::Solver
{
enum class PipelineCompletionStrategy
{
	ReturnFirstSuccess,
	WaitForAcceptableResult,
	WaitForAllResults
};

struct PipelineSolverParameters
{
	PipelineCompletionStrategy strategy { PipelineCompletionStrategy::ReturnFirstSuccess };
	double min_score_threshold { 0.25 };
};
}