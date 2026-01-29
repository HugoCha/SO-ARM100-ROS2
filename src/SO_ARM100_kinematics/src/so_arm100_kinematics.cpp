#include "so_arm100_kinematics.hpp"

#include <Eigen/Dense>

namespace SOArm100::Kinematics
{

// ------------------------------------------------------------

bool SOArm100AnalyticKinematicsPlugin::initialize(
	const rclcpp::Node::SharedPtr& node,
	const moveit::core::RobotModel& robot_model,
	const std::string& group_name,
	const std::string& base_frame,
	const std::vector< std::string >& tip_frames,
	double search_discretization )
{
	storeValues( robot_model, group_name, base_frame, tip_frames, search_discretization );
	return true;
}

// ------------------------------------------------------------

bool SOArm100AnalyticKinematicsPlugin::getPositionFK(
	const std::vector< std::string >& link_names,
	const std::vector< double >& joint_angles,
	std::vector< geometry_msgs::msg::Pose >& poses ) const
{
	return false;
}

// ------------------------------------------------------------

bool SOArm100AnalyticKinematicsPlugin::getPositionIK(
	const geometry_msgs::msg::Pose& ik_pose,
	const std::vector< double >& ik_seed_state,
	std::vector< double >& solution,
	moveit_msgs::msg::MoveItErrorCodes& error_code,
	const kinematics::KinematicsQueryOptions& options ) const
{
	return false;
}

// ------------------------------------------------------------

}
