#pragma once

#include "Global.hpp"
#include "Param.hpp"

namespace SOArm100::Kinematics::Params
{

inline const Param<double> Tolerance{
    "tolerance",
    error_tolerance
};

inline const Param<int> MaxIterations{
    "max_iterations",
    100
};

inline const Param<double> OrientationVsPosition {
/*
"Weight of orientation error vs position error
                * < 1.0: orientation has less importance than position
                * > 1.0: orientation has more importance than position
                * = 0.0: perform position-only IK",
*/
    "orientation_vs_position",
    1.0,
};

}