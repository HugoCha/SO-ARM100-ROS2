#pragma once

namespace SOArm100::Kinematics
{
enum class DLSSolverState
{
	Converged,
	Improving,
	BestPossible,
	Stalled,
	MaxRestart,
	MaxIterations,
	Failed,
	Unreachable
};
}