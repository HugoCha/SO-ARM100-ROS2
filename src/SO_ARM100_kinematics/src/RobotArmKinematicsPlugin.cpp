#include "RobotArmKinematicsPlugin.hpp"
#include "Global.hpp"
#include "Logger.hpp"

#include <chrono>
#include <Eigen/Dense>
#include <moveit/robot_model/robot_model.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

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
	const auto& chain = solver_.GetModel()->GetChain();
	link_names_ = chain->GetLinkNames();
	joint_names_ = chain->GetJointNames();

	return initialized_;
}

// ------------------------------------------------------------

bool RobotArmKinematicsPlugin::getPositionFK(
	const std::vector< std::string >& link_names,
	const std::vector< double >& joint_angles,
	std::vector< geometry_msgs::msg::Pose >& poses ) const
{
	geometry_msgs::msg::Pose tip_pose;
	bool success = solver_.ForwardKinematic( link_names, joint_angles, poses, tip_pose );

	std::stringstream log_ss;
	log_ss << "Joints " << ToVecXd( joint_angles ).transpose() << std::endl;
	RCLCPP_DEBUG( Logger::get(), "getPositionFK: Joints %s Tip Position is ( x:%f y:%f z:%f ) Orientation is ( x:%f y:%f z:%f w:%f )"
	              , log_ss.str().c_str()
	              , tip_pose.position.x, tip_pose.position.y, tip_pose.position.z
	              , tip_pose.orientation.x, tip_pose.orientation.y, tip_pose.orientation.z, tip_pose.orientation.w );

	return success;
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

	Eigen::Quaterniond q(
		ik_pose.orientation.w,
		ik_pose.orientation.x,
		ik_pose.orientation.y,
		ik_pose.orientation.z );

	Eigen::AngleAxisd aa( q );

	RCLCPP_INFO(
		Logger::get(),
		"IK request:"
		" position = [%f %f %f]"
		" orientation axis=[%f %f %f] angle=%f"
		" timeout=%f(s) approximate=%d",
		ik_pose.position.x,
		ik_pose.position.y,
		ik_pose.position.z,
		aa.axis().x(),
		aa.axis().y(),
		aa.axis().z(),
		aa.angle(),
		timeout,
		options.return_approximate_solution );

	do
	{
		auto result = solver_.InverseKinematic(
			ik_pose,
			ik_seed_state,
			consistency_limits,
			timeout_ms,
			error_tolerance,
			options.return_approximate_solution,
			solution );

		if ( result || options.return_approximate_solution )
		{
			if ( solution_callback )
			{
				solution_callback( ik_pose, solution, error_code );
				if ( error_code.val != error_code.SUCCESS )
					continue;
			}

			std::stringstream solution_ss;
			solution_ss << "Joints";
			for ( const auto& joint : solution )
				solution_ss << " " << joint;
			RCLCPP_DEBUG( Logger::get(), "Solved after %f < %f Error code: %d %s",
			              std::chrono::duration_cast< std::chrono::duration< double >>( std::chrono::steady_clock::now() - start_time ).count(),
			              timeout,
			              error_code.val,
			              solution_ss.str().c_str() );

			error_code.val = error_code.SUCCESS;
			return true;
		}
	}
	while ( !TimedOut( start_time, timeout ) );

	error_code.val = error_code.TIMED_OUT;
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

bool RobotArmKinematicsPlugin::TimedOut( std::chrono::time_point< std::chrono::steady_clock > start_time, double timeout )
{
	return std::chrono::duration_cast< std::chrono::duration< double >>( std::chrono::steady_clock::now() - start_time ).count() > timeout;
}

// ------------------------------------------------------------

}
