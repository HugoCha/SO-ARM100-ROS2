#pragma once

#include "Types.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/macros/class_forward.hpp>
#include <string>

namespace moveit::core
{
MOVEIT_CLASS_FORWARD( RobotModel );
MOVEIT_CLASS_FORWARD( JointModelGroup );
}

namespace SOArm100::Kinematics
{
class WorkspaceFilter
{
public:
WorkspaceFilter() = default;
WorkspaceFilter(
	const moveit::core::RobotModelConstPtr& robot_model,
	const std::string& group_name,
	const std::string& base_frame_name );

[[nodiscard]] bool IsUnreachable( const geometry_msgs::msg::Pose& target_pose ) const;

protected:
const moveit::core::JointModelGroup* joints_;
const std::string base_frame_name_;

virtual void ComputeWorkspace(
	const moveit::core::JointModelGroup* joints,
	const std::string& base_frame_name );

[[nodiscard]] virtual double ComputeMaxReach(
	const moveit::core::JointModelGroup* joints,
	const std::string& base_frame_name ) const;

[[nodiscard]] virtual double ComputeMinReach(
	const moveit::core::JointModelGroup* joints,
	const std::string& base_frame_name ) const;

[[nodiscard]] const Mat4d GetBaseFrame() const;

private:
double min_reach_;
double max_reach_;
};
}