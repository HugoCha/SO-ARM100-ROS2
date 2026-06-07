#pragma once

#include "Global.hpp"
#include <limits>

namespace SOArm100::Kinematics::Solver
{
struct SolverHistory
{
	int restart_counter{ 0 };
	int stalled_error_cnt{ 0 };
	int last_non_stalled_error_idx{ 0 };
	double last_non_stalled_error{ std::numeric_limits< double >::infinity() };
	double best_error{ std::numeric_limits< double >::infinity() };
	VecXd best_joints;
};
}