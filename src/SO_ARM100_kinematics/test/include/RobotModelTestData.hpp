#pragma once
#include <moveit/robot_model/robot_model.hpp>

namespace SOArm100::Kinematics::Test::Data
{
moveit::core::RobotModelConstPtr GetRevoluteOnlyRobot();
}