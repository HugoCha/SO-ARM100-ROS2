#pragma once

#include "Global.hpp"

#include "Heuristic/IKPresolution.hpp"
#include "IKSolverState.hpp"

namespace SOArm100::Kinematics::Solver
{
struct IKSolution
{
	IKSolverState state = IKSolverState::NotRun;
	VecXd joints;
	double error;
	int iterations = 0;
	double score = std::numeric_limits< double >::infinity();

	[[nodiscard]] bool Success() const {
		return state == IKSolverState::Converged;
	}

	operator Heuristic::IKPresolution() const
	{
		return { joints, ToIKHeuristicState( state ), error, iterations };
	}
};
}