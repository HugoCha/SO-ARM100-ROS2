#pragma once

#include "Global.hpp"

#include "Twist.hpp"

#include <moveit/robot_model/robot_model.hpp>
#include <vector>

namespace SOArm100::Kinematics::Test::Data
{
moveit::core::RobotModelConstPtr GetRevoluteOnlyRobot();
Mat4d GetRevoluteOnlyRobotTransform( double theta1, double theta2, double theta3 );
std::vector< TwistConstPtr > GetRevoluteOnlyRobotTwists();
MatXd GetRevoluteOnlyRobotJacobian( double theta1, double theta2, double theta3 );
}