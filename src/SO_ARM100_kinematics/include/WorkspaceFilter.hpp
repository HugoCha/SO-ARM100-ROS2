#pragma once

#include "Global.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/macros/class_forward.hpp>
#include <span>

namespace moveit::core
{
MOVEIT_CLASS_FORWARD( JointModel );
}

namespace SOArm100::Kinematics
{
class WorkspaceFilter
{
public:
WorkspaceFilter() = default;
WorkspaceFilter( const std::span< const moveit::core::JointModel* const >& joint_models );

[[nodiscard]] bool IsUnreachable( const geometry_msgs::msg::Pose& target_pose ) const;

protected:
const std::span< const moveit::core::JointModel* const > joint_models_;

virtual void ComputeWorkspace( const std::span< const moveit::core::JointModel* const >& joint_models );

[[nodiscard]] virtual double ComputeMaxReach( const std::span< const moveit::core::JointModel* const >& joint_models ) const;
[[nodiscard]] virtual double ComputeMinReach( const std::span< const moveit::core::JointModel* const >& joint_models ) const;

[[nodiscard]] const Mat4d GetBaseFrame() const;

private:
double min_reach_;
double max_reach_;
};
}