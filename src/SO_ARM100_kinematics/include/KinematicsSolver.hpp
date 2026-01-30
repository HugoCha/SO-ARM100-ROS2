#pragma once

#include "Twist.hpp"
#include "Types.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/macros/class_forward.hpp>
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
	geometry_msgs::msg::Pose& pose );

virtual bool InverseKinematic(
	const geometry_msgs::msg::Pose& target_pose,
	std::vector< double >& joint_angles ) = 0;

protected:
moveit::core::RobotModelConstPtr robot_model_;
std::vector< Twist > twists_;
Mat4d home_configuration_;

bool ForwardKinematic( const VecXd& joint_angles, Mat4d& pose );
bool CheckLimits( const std::span< const double >& joint_angles );
};
}
