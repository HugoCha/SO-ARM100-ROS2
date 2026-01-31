#pragma once

#include "Twist.hpp"
#include "Types.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/macros/class_forward.hpp>
#include <moveit/robot_model/joint_model_group.hpp>
#include <span>

namespace moveit::core
{
MOVEIT_CLASS_FORWARD( RobotModel );
}

namespace SOArm100::Kinematics
{
class KinematicsSolver
{
public:
KinematicsSolver();
~KinematicsSolver();

void Initialize(
	const moveit::core::RobotModelConstPtr& robot_model,
	const std::string& group_name,
	const std::string& base_frame,
	const std::vector< std::string >& tip_frames,
	double search_discretization );

bool ForwardKinematic(
	const std::span< const double >& joint_angles,
	geometry_msgs::msg::Pose& pose ) const;

bool ForwardKinematic(
	const VecXd& joint_angles,
	Mat4d& pose ) const;

virtual bool InverseKinematic(
	const geometry_msgs::msg::Pose& target_pose,
	const std::span< const double >& seed_joints,
	std::vector< double >& joints ) const = 0;

protected:
const moveit::core::JointModelGroup* joint_model_;
std::vector< Twist > twists_;
Mat4d home_configuration_;

[[nodiscard]] bool AreValidInitializeParameters(
	const moveit::core::RobotModelConstPtr& robot_model,
	const std::string& group_name,
	const std::string& base_frame,
	const std::vector< std::string >& tip_frames,
	double search_discretization ) const noexcept;
bool CheckLimits( const std::span< const double >& joint_angles ) const;
};
}
