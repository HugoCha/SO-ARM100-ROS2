#pragma once

namespace SOArm100::Kinematics
{
class AdaptativeStep
{
public:
static double ReachableRatio(
    double min_step,
    double max_step,
    double error,
    double reachable_error );
};
}