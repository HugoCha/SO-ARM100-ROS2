#pragma once

namespace SOArm100::Kinematics
{
enum class NumericSolverState
{
	Converged,
	Improving,
	BestPossible,
	Stalled,
	MaxRestart,
	MaxIterations,
	Failed
};
}