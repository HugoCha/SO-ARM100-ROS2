#pragma once

#include "Global.hpp"

#include <moveit/macros/class_forward.hpp>

namespace moveit::core
{
MOVEIT_CLASS_FORWARD( RobotModel );
}

namespace SOArm100::Kinematics
{
class JointChain;
};

namespace SOArm100::Kinematics::Test::Data
{
moveit::core::RobotModelConstPtr GetRevoluteOnlyRobot();
Mat4d GetRevoluteOnlyRobotTransform( double theta1, double theta2, double theta3 );
const JointChain GetRevoluteOnlyRobotJointChain();
MatXd GetRevoluteOnlyRobotJacobian( double theta1, double theta2, double theta3 );
}