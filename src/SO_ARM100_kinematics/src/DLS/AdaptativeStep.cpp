#include "DLS/AdaptativeStep.hpp"

#include <algorithm>

namespace SOArm100::Kinematics
{

// ------------------------------------------------------------

double AdaptativeStep::ReachableRatio(
	double min_step,
	double max_step,
	double error,
	double reachable_error )
{
	double reachable_ratio = std::clamp( reachable_error / error, 0.0, 1.0 );
	return std::max( min_step, max_step * reachable_ratio );
}

// ------------------------------------------------------------

}