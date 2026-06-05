#pragma once

#include "Global.hpp"

namespace SOArm100::Kinematics::Solver
{
struct UniversalSolution {
	Vec2d angles;          ///< physical joint angles (θ₁, θ₂) [rad]
	double cost;           ///< optimizer cost at solution
	double fk_error;       ///< ‖FK(θ)·p_tcp_local − p_target‖ [m]
	bool reachable;        ///< all joints within limits?
};
}