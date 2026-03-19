#pragma once

#include "Heuristic/IKHeuristicState.hpp"

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

inline Heuristic::IKHeuristicState ToIKHeuristicState( const IKSolverState& state )
{
	switch ( state )
	{
	case IKSolverState::Converged:
		return Heuristic::IKHeuristicState::Success;
	case IKSolverState::BestPossible:
		return Heuristic::IKHeuristicState::PartialSuccess;
	default:
		return Heuristic::IKHeuristicState::Fail;
	}
}
}