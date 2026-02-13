#pragma once

#include "Global.hpp"

#include "Twist.hpp"
#include <memory>

namespace SOArm100::Kinematics
{
struct BaseJointModel
{
Vec3d reference_direction;
TwistConstPtr twist;
};

using BaseJointModelUniqueConstPtr = std::unique_ptr< const BaseJointModel >;
}