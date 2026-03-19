#pragma once

#include "JointGroup.hpp"

#include <optional>

namespace SOArm100::Kinematics::Model
{
struct KinematicTopology
{
std::optional< JointGroup > base{std::nullopt};
std::optional< JointGroup > elbow{std::nullopt};
std::optional< JointGroup > wrist{std::nullopt};
};
}