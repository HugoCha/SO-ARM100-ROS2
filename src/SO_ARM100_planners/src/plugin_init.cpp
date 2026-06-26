#include <pluginlib/class_list_macros.hpp>

#include "RobotArmPlannersPlugin.hpp"

PLUGINLIB_EXPORT_CLASS(
	SOArm100::Planners::RobotArmPlannersPlugin,
	planning_interface::PlannerManager )