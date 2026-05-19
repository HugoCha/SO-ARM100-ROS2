#pragma once

#include "Global.hpp"

namespace SOArm100::Kinematics::Solver
{
struct SphericalSolution {
	Vec3d angles;          ///< physical joint angles (θ₁, θ₂, θ₃) [rad]
	double phi;            ///< optimal free angle φ [rad]
	double cost;           ///< optimizer cost at solution
	double fk_error;       ///< ‖FK(θ)·p_tcp_local − p_target‖ [m]
	double singularity_margin; ///< 0=singular, 1=far (cos θ₂ in canonical frame)
	bool reachable;        ///< all joints within limits?
	bool near_singular;    ///< middle joint near gimbal lock?
};
}