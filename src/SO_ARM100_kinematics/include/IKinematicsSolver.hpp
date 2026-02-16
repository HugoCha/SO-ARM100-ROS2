#pragma once

#include "Global.hpp"

#include <span>

namespace SOArm100::Kinematics
{
struct SolverResult;

class IKinematicsSolver
{
public:
virtual ~IKinematicsSolver() = default;

[[nodiscard]] virtual SolverResult IK(
	const Mat4d& target_pose,
	const std::span< const double >& seed_joints,
	double search_discretization ) const = 0;
};
}