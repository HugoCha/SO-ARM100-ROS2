#pragma once

#include "Global.hpp"

#include "JointState.hpp"

namespace SOArm100::Kinematics::Model
{
struct ArticulationState
{
Pose center;
Quaternion rotation;
std::vector< JointState > joint_states;
};
}