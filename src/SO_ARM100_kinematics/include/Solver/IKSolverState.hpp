#pragma once

namespace SOArm100::Kinematics::Solver
{
enum class IKSolverState
{
	Converged,
	BestPossible,
	MaxRestart,
	MaxIterations,
    NotRun,
	Unreachable
};
}