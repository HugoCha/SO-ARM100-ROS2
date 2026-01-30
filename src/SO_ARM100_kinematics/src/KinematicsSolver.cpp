#include "KinematicsSolver.hpp"

#include "Converter.hpp"
#include "MatrixExponential.hpp"

#include <moveit/robot_model/joint_model.hpp>
#include <moveit/robot_model/link_model.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <rclcpp/logger.hpp>
#include <vector>

namespace SOArm100::Kinematics
{

static rclcpp::Logger get_logger()
{
	static rclcpp::Logger logger = rclcpp::get_logger( "KinematicsSolver" );
	return logger;
}

// ------------------------------------------------------------

KinematicsSolver::KinematicsSolver()
{
}

// ------------------------------------------------------------

KinematicsSolver::~KinematicsSolver()
{
}

// ------------------------------------------------------------

void KinematicsSolver::Initialize(
	const moveit::core::RobotModelConstPtr& robot_model,
	const std::string& group_name,
	const std::string& base_frame,
	const std::vector< std::string >& tip_frames,
	double search_discretization )
{
	RCLCPP_INFO( get_logger(), "Initializing KinematicsSolver for group: %s",
	             group_name.c_str());
	robot_model_ = robot_model;

	moveit::core::RobotState state( robot_model_ );
	state.setToDefaultValues();

	auto joint_group = robot_model_->getJointModelGroup( group_name );
	if ( !joint_group )
	{
		RCLCPP_ERROR( get_logger(), "Joint model group %s not found in robot model.",
		              group_name.c_str());
		return;
	}

	const auto& joint_models = joint_group->getActiveJointModels();

	home_configuration_ = state.getGlobalLinkTransform( tip_frames[0] ).matrix();
	twists_.clear();
	for ( const auto& joint_model : joint_models )
	{
		if ( joint_model->getType() != moveit::core::JointModel::REVOLUTE &&
		     joint_model->getType() != moveit::core::JointModel::FIXED )
		{
			RCLCPP_WARN( get_logger(),
			             "Joint %s is not fixed, revolute or prismatic. Skipping.",
			             joint_model->getName().c_str());
			continue;
		}

		Eigen::Vector3d axis;
		if ( joint_model->getType() == moveit::core::JointModel::FIXED )
		{
			axis = Eigen::Vector3d::Zero();
		}
		else
		if ( joint_model->getType() == moveit::core::JointModel::REVOLUTE )
		{
			const auto* revolute_joint_model =
				static_cast< const moveit::core::RevoluteJointModel* >( joint_model );
			axis = revolute_joint_model->getAxis();
		}
		auto joint_transform =
			state.getGlobalLinkTransform( joint_model->getParentLinkModel());
		Eigen::Matrix3d R = joint_transform.rotation();
		Eigen::Vector3d point_on_axis_world = joint_transform.translation();
		Eigen::Vector3d axis_world = R * axis;

		auto twist = Twist( axis_world, point_on_axis_world );
		twists_.push_back( twist );
	}
}

// ------------------------------------------------------------

bool KinematicsSolver::ForwardKinematic(
	const std::vector< double >& joint_angles,
	geometry_msgs::msg::Pose& pose )
{
	Mat4d T_end;
	VecXd joints = ToVecXd( joint_angles );
	if ( ForwardKinematic( joints, T_end ) )
	{
		pose = ToPoseMsg( T_end );
		return true;
	}
	return false;
}

// ------------------------------------------------------------

bool KinematicsSolver::ForwardKinematic(
	const VecXd& joint_angles,
	Mat4d& pose )
{
	if ( !robot_model_ )
	{
		RCLCPP_ERROR( get_logger(), "Robot model not initialized." );
		return false;
	}

	if ( joint_angles.size() != twists_.size() )
	{
		RCLCPP_ERROR(
			get_logger(),
			"Joint angles size (%zu) does not match number of twists (%zu).",
			joint_angles.size(), twists_.size());
		return false;
	}

	Eigen::Matrix4d T_end = Eigen::Matrix4d::Identity();
	for ( size_t i = 0; i < twists_.size(); ++i )
	{
		Eigen::Matrix4d exp_twist = MatrixExponential( twists_[i], joint_angles[i] );
		T_end = T_end * exp_twist;
	}
	pose = T_end * home_configuration_;

	return true;
}

// ------------------------------------------------------------

bool KinematicsSolver::CheckLimits( const std::vector< double >& joint_angles )
{
	if ( !robot_model_ )
	{
		return false;
	}

	const auto& active_joints = robot_model_->getActiveJointModels();
	int joint_index = 0;
	int active_joint_count = active_joints.size();
	if ( active_joint_count != joint_angles.size() )
	{
		return false;
	}

	while ( joint_index < active_joint_count )
	{
		const auto& joint_model = active_joints[joint_index];
		if ( joint_model->getType() !=
		     moveit::core::JointModel::JointType::REVOLUTE )
		{
			return false;
		}

		const auto& joint_bound = joint_model->getVariableBounds();
		if ( !joint_bound.empty() )
		{
			auto joint_angle = joint_angles[joint_index];
			if ( joint_angle > joint_bound[0].max_position_ ||
			     joint_angle < joint_bound[0].min_position_ )
			{
				return false;
			}
		}
		joint_index++;
	}
	return true;
}

// ------------------------------------------------------------

} // namespace SOArm100::Kinematics
