#include "RobotArmKinematicsPlugin.hpp"
#include "Global.hpp"

#include <chrono>
#include <Eigen/Dense>
#include <moveit/robot_model/robot_model.hpp>
#include <rclcpp/rclcpp.hpp>

namespace SOArm100::Kinematics
{

// ------------------------------------------------------------

bool RobotArmKinematicsPlugin::initialize(
	const rclcpp::Node::SharedPtr& node,
	const moveit::core::RobotModel& robot_model,
	const std::string& group_name,
	const std::string& base_frame,
	const std::vector< std::string >& tip_frames,
	double search_discretization )
{
	storeValues( robot_model, group_name, base_frame, tip_frames, search_discretization );
	initialized_ = solver_.Initialize( robot_model_, group_name, base_frame, tip_frames, search_discretization );
	return initialized_;
}

// ------------------------------------------------------------

bool RobotArmKinematicsPlugin::getPositionFK(
	const std::vector< std::string >& link_names,
	const std::vector< double >& joint_angles,
	std::vector< geometry_msgs::msg::Pose >& poses ) const
{
	geometry_msgs::msg::Pose tip_pose;
	return solver_.ForwardKinematic( link_names, joint_angles, poses, tip_pose );
}

// ------------------------------------------------------------

bool RobotArmKinematicsPlugin::searchPositionIK(
	const geometry_msgs::msg::Pose& ik_pose,
	const std::vector< double >& ik_seed_state,
	double timeout,
	const std::vector< double >& consistency_limits,
	std::vector< double >& solution,
	const IKCallbackFn& solution_callback,
	moveit_msgs::msg::MoveItErrorCodes& error_code,
	const kinematics::KinematicsQueryOptions& options ) const
{
	auto start_time = std::chrono::steady_clock::now();
	long timeout_ms = timeout * 1000;

	if ( !initialized_ )
	{
		RCLCPP_ERROR( Logger::get(), "kinematics solver not initialized" );
		error_code.val = error_code.NO_IK_SOLUTION;
		return false;
	}

	if ( ik_seed_state.size() != solver_.GetModel()->GetChain()->GetJointCount() )
	{
		RCLCPP_ERROR( Logger::get(),
		              "Seed state must have size  %zu, instead of size  %zu",
		              solver_.GetModel()->GetChain()->GetJointCount(),
		              ik_seed_state.size() );
		error_code.val = error_code.NO_IK_SOLUTION;
		return false;
	}

	if ( !consistency_limits.empty() )
	{
		if ( consistency_limits.size() != solver_.GetModel()->GetChain()->GetJointCount() )
		{
			RCLCPP_ERROR( Logger::get(),
			              "Consistency limits must be empty or have size %zu  instead of size %zu",
			              solver_.GetModel()->GetChain()->GetJointCount(),
			              consistency_limits.size() );

			error_code.val = error_code.NO_IK_SOLUTION;
			return false;
		}
	}

	RCLCPP_DEBUG( Logger::get(), "searchPositionIK: Position request is ( x:%f y:%f z:%f ) Orientation is ( x:%f y:%f z:%f w:%f )"
	              , ik_pose.position.x, ik_pose.position.y, ik_pose.position.z
	              , ik_pose.orientation.x, ik_pose.orientation.y, ik_pose.orientation.z, ik_pose.orientation.w );

	do
	{
		auto result = solver_.InverseKinematic(
			ik_pose,
			ik_seed_state,
			consistency_limits,
			timeout_ms,
			error_tolerance,
			solution );

		if ( result )
		{
			if ( solution_callback )
			{
				solution_callback( ik_pose, solution, error_code );
				if ( error_code.val != error_code.SUCCESS )
					continue;
			}

			RCLCPP_DEBUG( Logger::get(), "Solved after %f < %f",
			              std::chrono::duration_cast< std::chrono::duration< double >>( std::chrono::steady_clock::now() - start_time ).count(),
			              timeout );

			return true;
		}

	}
	while ( !TimedOut( start_time, timeout_ms ) );

	return false;
}

// ------------------------------------------------------------

bool RobotArmKinematicsPlugin::getPositionIK(
	const geometry_msgs::msg::Pose& ik_pose,
	const std::vector< double >& ik_seed_state,
	std::vector< double >& solution,
	moveit_msgs::msg::MoveItErrorCodes& error_code,
	const kinematics::KinematicsQueryOptions& options ) const
{
	std::vector< double > consistency_limits;
	return searchPositionIK(
		ik_pose,
		ik_seed_state,
		0.0,
		consistency_limits,
		solution,
		IKCallbackFn(),
		error_code,
		options );
}

bool RobotArmKinematicsPlugin::searchPositionIK( const geometry_msgs::msg::Pose& ik_pose, const std::vector< double >& ik_seed_state,
                                                 double timeout, std::vector< double >& solution,
                                                 moveit_msgs::msg::MoveItErrorCodes& error_code,
                                                 const kinematics::KinematicsQueryOptions& options ) const
{
	std::vector< double > consistency_limits;

	return searchPositionIK(
		ik_pose,
		ik_seed_state,
		timeout,
		consistency_limits,
		solution,
		IKCallbackFn(),
		error_code,
		options );
}

bool RobotArmKinematicsPlugin::searchPositionIK( const geometry_msgs::msg::Pose& ik_pose, const std::vector< double >& ik_seed_state,
                                                 double timeout, const std::vector< double >& consistency_limits,
                                                 std::vector< double >& solution, moveit_msgs::msg::MoveItErrorCodes& error_code,
                                                 const kinematics::KinematicsQueryOptions& options ) const
{
	return searchPositionIK(
		ik_pose,
		ik_seed_state,
		timeout,
		consistency_limits,
		solution,
		IKCallbackFn(),
		error_code,
		options );
}

bool RobotArmKinematicsPlugin::searchPositionIK( const geometry_msgs::msg::Pose& ik_pose, const std::vector< double >& ik_seed_state,
                                                 double timeout, std::vector< double >& solution,
                                                 const IKCallbackFn& solution_callback,
                                                 moveit_msgs::msg::MoveItErrorCodes& error_code,
                                                 const kinematics::KinematicsQueryOptions& options ) const
{
	std::vector< double > consistency_limits;
	return searchPositionIK(
		ik_pose,
		ik_seed_state,
		timeout,
		consistency_limits,
		solution,
		solution_callback,
		error_code,
		options );
}

// ------------------------------------------------------------

bool RobotArmKinematicsPlugin::TimedOut( std::chrono::time_point< std::chrono::steady_clock > start_time, long timeout_ms )
{
	return std::chrono::duration_cast< std::chrono::duration< double >>( std::chrono::steady_clock::now() - start_time ).count() > timeout_ms;
}

// ------------------------------------------------------------

}
