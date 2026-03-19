#pragma once

#include "Global.hpp"

namespace SOArm100::Kinematics::Utils
{
enum class DistanceType
{
	Euclidean,
	Chebysev,
	Manhattan
};

double Distance(
	const VecXd& p1,
	const VecXd& p2,
	DistanceType type = DistanceType::Euclidean );
}