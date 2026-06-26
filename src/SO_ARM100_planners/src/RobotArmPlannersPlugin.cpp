#include "RobotArmPlannersPlugin.hpp"

namespace  SOArm100::Planners
{

// ------------------------------------------------------------

std::string RobotArmPlannersPlugin::getDescription() const
{
	return "";
}

// ------------------------------------------------------------

void RobotArmPlannersPlugin::getPlanningAlgorithms( std::vector< std::string >& algs ) const
{
	return;
}

// ------------------------------------------------------------

planning_interface::PlanningContextPtr RobotArmPlannersPlugin::getPlanningContext( const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                                                   const planning_interface::MotionPlanRequest& req,
                                                                                   moveit_msgs::msg::MoveItErrorCodes& error_code ) const
{
	return nullptr;
}

// ------------------------------------------------------------

bool RobotArmPlannersPlugin::canServiceRequest( const planning_interface::MotionPlanRequest& req ) const
{
	return false;
}

// ------------------------------------------------------------

}