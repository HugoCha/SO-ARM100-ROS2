#pragma once

namespace SOArm100::Kinematics
{
enum class DLSSolverState
{
	Converged,
	Improving,
	BestPossible,
	Stalled,
	MaxIterations,
	Failed,
	Unreachable
};
}