#pragma once
#include "Twist.hpp"

#include <moveit/robot_model/robot_model.hpp>
#include <vector>

namespace SOArm100::Kinematics::Test::Data
{
moveit::core::RobotModelConstPtr GetRevoluteOnlyRobot();
Eigen::Matrix4d GetRevoluteOnlyRobotTransform( double theta1, double theta2, double theta3 );
std::vector< Twist > GetRevoluteOnlyRobotTwists();
Eigen::MatrixXd GetRevoluteOnlyRobotJacobian( double theta1, double theta2, double theta3 );
}