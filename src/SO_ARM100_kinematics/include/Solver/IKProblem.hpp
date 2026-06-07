#pragma once

#include "Global.hpp"

namespace SOArm100::Kinematics::Solver
{
struct IKProblem
{
	Mat4d target;

	VecXd seed;
	VecXd consistency;

	double tolerance;

	long timeout_ms;

	bool CanReSeed() const {
		return timeout_ms != 0;
	}
};
}