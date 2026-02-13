#pragma once

#include "Global.hpp"

#include "Twist.hpp"

#include <memory>
#include <span>

#include <moveit/macros/class_forward.hpp>

namespace moveit::core
{
MOVEIT_CLASS_FORWARD( JointModel );
}

namespace SOArm100::Kinematics
{
struct NumericJointsModel
{
int start_index{0};
int count{0};
std::span< const moveit::core::JointModel* const > numeric_joints;
std::span< TwistConstPtr > twists{};
Mat4d home_configuration{ Mat4d::Identity() };
};

using NumericJointsModelUniqueConstPtr = std::unique_ptr< const NumericJointsModel >;
}