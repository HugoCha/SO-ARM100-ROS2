#pragma once

#include "Heuristic/IKPresolutionBranch.hpp"
#include "IKHeuristicState.hpp"

namespace SOArm100::Kinematics::Heuristic
{
struct IKPresolution
{
	std::vector< IKPresolutionBranch > branches;
	IKHeuristicState state;

	bool Success() const {
		return state == IKHeuristicState::Success;
	}

	bool PartialOrSuccess() const {
		return state == IKHeuristicState::PartialSuccess || state == IKHeuristicState::Success;
	}

	bool Fail() const {
		return state == IKHeuristicState::Fail;
	}
};
}