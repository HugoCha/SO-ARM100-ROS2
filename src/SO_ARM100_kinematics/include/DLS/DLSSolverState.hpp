#pragma once

namespace SOArm100::Kinematics::Solver
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