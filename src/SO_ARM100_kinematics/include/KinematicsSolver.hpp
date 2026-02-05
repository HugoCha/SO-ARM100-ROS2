#pragma once

#include "Twist.hpp"
#include "Types.hpp"
#include "WorkspaceFilter.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <memory>
#include <moveit/macros/class_forward.hpp>
#include <span>

namespace moveit::core
{
MOVEIT_CLASS_FORWARD( JointModelGroup );
MOVEIT_CLASS_FORWARD( RobotModel );
}

namespace SOArm100::Kinematics
{
class KinematicsSolver
{
public:
KinematicsSolver();
~KinematicsSolver() = default;

void Initialize(
	const moveit::core::RobotModelConstPtr& robot_model,
	const std::string& group_name,
	const std::string& base_frame,
	const std::vector< std::string >& tip_frames,
	double search_discretization );

[[nodiscard]] bool ForwardKinematic(
	const std::span< const double >& joint_angles,
	geometry_msgs::msg::Pose& pose ) const;

[[nodiscard]] bool InverseKinematic(
	const geometry_msgs::msg::Pose& target_pose,
	const std::span< const double >& seed_joints,
	std::vector< double >& joints ) const;

protected:
const moveit::core::JointModelGroup* joint_model_;
std::unique_ptr< std::vector< Twist >> p_twists_;
std::unique_ptr< const Mat4d > p_home_configuration_;
std::unique_ptr< WorkspaceFilter > p_workspace_filter;

[[nodiscard]] bool ForwardKinematic(
	const VecXd& joint_angles,
	Mat4d& pose ) const;

[[nodiscard]] virtual bool InverseKinematic(
	const Mat4d& target_pose,
	const std::span< const double >& seed_joints,
	VecXd& joints ) const = 0;

[[nodiscard]] bool AreValidInitializeParameters(
	const moveit::core::RobotModelConstPtr& robot_model,
	const std::string& group_name,
	const std::string& base_frame,
	const std::vector< std::string >& tip_frames,
	double search_discretization ) const noexcept;
[[nodiscard]] bool CheckLimits( const std::span< const double >& joint_angles ) const;
};
}
