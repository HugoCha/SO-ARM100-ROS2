#pragma once

#include "Global.hpp"

#include <moveit/macros/class_forward.hpp>

namespace moveit::core
{
MOVEIT_CLASS_FORWARD( RobotModel );
}

namespace SOArm100::Kinematics::Model
{
class JointChain;
};

namespace SOArm100::Kinematics::Test::Data
{
moveit::core::RobotModelConstPtr GetRevoluteOnlyRobot();
Mat4d GetRevoluteOnlyRobotHome();
const Mat4d GetRevoluteOnlyRobotT01( double theta1 );
const Mat4d GetRevoluteOnlyRobotT12( double theta2 );
const Mat4d GetRevoluteOnlyRobotT23( double theta3 );
Mat4d GetRevoluteOnlyRobotTransform( double theta1, double theta2, double theta3 );
Model::JointChain GetRevoluteOnlyRobotJointChain();
MatXd GetRevoluteOnlyRobotJacobian( double theta1, double theta2, double theta3 );

Model::JointChain Create5DofRobotJointChain();
}