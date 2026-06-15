#pragma once

#include "Global.hpp"
#include <limits>

namespace SOArm100::Kinematics::Heuristic
{
struct IKPresolutionBranch
{
	VecXd joints;
	double error{ std::numeric_limits< double >::infinity() };
	double cost{ std::numeric_limits< double >::infinity() };
};
}