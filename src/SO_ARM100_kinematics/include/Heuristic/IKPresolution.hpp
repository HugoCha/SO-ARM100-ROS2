#pragma once

#include "Global.hpp"

#include "IKHeuristicState.hpp"

namespace SOArm100::Kinematics::Heuristic
{
struct IKPresolution
{
	VecXd joints;
	IKHeuristicState state;
	double error;
	int iterations { 0 };

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