#include <pluginlib/class_list_macros.hpp>
#include <moveit/kinematics_base/kinematics_base.hpp>
#include "so_arm100_kinematics.hpp"

PLUGINLIB_EXPORT_CLASS(
	SOArm100::Kinematics::SOArm100AnalyticKinematicsPlugin,
	kinematics::KinematicsBase
	)
