#pragma once

#include "Global.hpp"
#include "Model/JointState.hpp"

namespace SOArm100::Kinematics::Model
{
struct ArticulationState
{
Pose center;
Quaternion rotation;
std::vector< JointState > joint_states;
};
}