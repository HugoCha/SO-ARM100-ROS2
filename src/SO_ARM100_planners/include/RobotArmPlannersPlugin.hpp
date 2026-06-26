#pragma once

#include <moveit/planning_interface/planning_interface.hpp>

namespace SOArm100::Planners 
{
class RobotArmPlannersPlugin : public planning_interface::PlannerManager
{
public:
virtual std::string getDescription() const override;
virtual void getPlanningAlgorithms(std::vector<std::string>& algs) const override;
virtual planning_interface::PlanningContextPtr getPlanningContext(const planning_scene::PlanningSceneConstPtr& planning_scene,
    const planning_interface::MotionPlanRequest& req,
    moveit_msgs::msg::MoveItErrorCodes& error_code) const override;
virtual bool canServiceRequest(const planning_interface::MotionPlanRequest& req) const override;
};
}