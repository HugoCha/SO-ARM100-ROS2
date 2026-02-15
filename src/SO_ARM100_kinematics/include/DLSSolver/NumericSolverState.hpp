#pragma once

namespace SOArm100::Kinematics
{
enum class NumericSolverState
{
	Converged,
	Improving,
	Stalled,
	MaxIterations,
	Failed
};
}