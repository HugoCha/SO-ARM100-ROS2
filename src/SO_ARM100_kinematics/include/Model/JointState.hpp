#pragma once

#include "Pose.hpp"

namespace SOArm100::Kinematics::Model
{
struct JointState
{
Pose pose;
double value;
};
}