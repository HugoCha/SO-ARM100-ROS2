#include "RobotArmKinematicsPlugin.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <moveit/kinematics_base/kinematics_base.hpp>

PLUGINLIB_EXPORT_CLASS(
	SOArm100::Kinematics::RobotArmKinematicsPlugin,
	kinematics::KinematicsBase
	)
