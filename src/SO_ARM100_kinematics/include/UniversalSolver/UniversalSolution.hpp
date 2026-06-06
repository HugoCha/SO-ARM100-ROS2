#pragma once

#include "Global.hpp"

namespace SOArm100::Kinematics::Solver
{
struct UniversalSolution {
	Vec2d angles;
	Mat3d local_rotation;
	double cost;
	double fk_error;
	bool reachable;
};
}