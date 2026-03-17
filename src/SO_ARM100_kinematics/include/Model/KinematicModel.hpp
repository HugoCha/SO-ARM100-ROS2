#pragma once

#include "JointChain.hpp"
#include <memory>

namespace SOArm100::Kinematics
{
struct KinematicModel
{
JointChainConstPtr chain;
Mat4d home;
};

using KinematicModelConstPtr = std::shared_ptr< const KinematicModel >;
}