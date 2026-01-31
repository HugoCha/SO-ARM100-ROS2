#include "KinematicsSolver.hpp"

#include "Converter.hpp"
#include "MatrixExponential.hpp"

#include <cassert>
#include <moveit/robot_model/joint_model.hpp>
#include <moveit/robot_model/link_model.hpp>
#include <moveit/robot_model/prismatic_joint_model.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <rclcpp/logger.hpp>

namespace SOArm100::Kinematics
{

// ------------------------------------------------------------

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
	             group_name.c_str() );

	if ( !AreValidInitializeParameters( robot_model, group_name, base_frame, tip_frames, search_discretization ) )
	{
		return;
	}

	moveit::core::RobotState state( robot_model );
	state.setToDefaultValues();
	joint_model_ = robot_model->getJointModelGroup( group_name );
	const auto& joint_models = joint_model_->getActiveJointModels();
	home_configuration_ = state.getGlobalLinkTransform( tip_frames[0] ).matrix();

	twists_.clear();
	twists_.reserve( joint_models.size() );

	for ( const auto* joint_model : joint_models )
	{
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
		else
		if ( joint_model->getType() == moveit::core::JointModel::PRISMATIC )
		{
			const auto* revolute_joint_model =
				static_cast< const moveit::core::PrismaticJointModel* >( joint_model );
			axis = revolute_joint_model->getAxis();
		}

		const auto& joint_transform =
			state.getGlobalLinkTransform( joint_model->getParentLinkModel() );

		const Eigen::Vector3d axis_world = joint_transform.rotation() * axis;
		const Eigen::Vector3d point_on_axis_world = joint_transform.translation();

		twists_.emplace_back( axis_world, point_on_axis_world );
	}
}

// ------------------------------------------------------------

bool KinematicsSolver::ForwardKinematic(
	const std::span< const double >& joint_angles,
	geometry_msgs::msg::Pose& pose ) const
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
	Mat4d& pose ) const
{
	if ( !joint_model_ )
	{
		RCLCPP_ERROR( get_logger(), "Joint model not initialized." );
		return false;
	}

	if ( joint_angles.size() != twists_.size() )
	{
		RCLCPP_ERROR(
			get_logger(),
			"Joint angles size (%zu) does not match number of twists (%zu).",
			joint_angles.size(), twists_.size() );
		return false;
	}

	pose.setIdentity();
	for ( size_t i = 0; i < twists_.size(); ++i )
	{
		pose *= static_cast< Mat4d >( MatrixExponential( twists_[i], joint_angles[i] ) );
	}
	pose *= home_configuration_;

	return true;
}

// ------------------------------------------------------------

bool KinematicsSolver::AreValidInitializeParameters(
	const moveit::core::RobotModelConstPtr& robot_model,
	const std::string& group_name,
	const std::string& base_frame,
	const std::vector< std::string >& tip_frames,
	double search_discretization ) const noexcept
{
	if ( !robot_model )
	{
		RCLCPP_ERROR( get_logger(), "Robot Model is null." );
		return false;
	}

	const auto* joint_model = robot_model->getJointModelGroup( group_name );
	if ( !joint_model )
	{
		RCLCPP_ERROR( get_logger(), "Joint model group %s not found in robot model.",
		              group_name.c_str() );
		return false;
	}
	if ( !joint_model->isChain() )
	{
		RCLCPP_ERROR( get_logger(), "Joint model group %s is not a chain.", group_name.c_str() );
		return false;
	}
	if ( !joint_model->isSingleDOFJoints() )
	{
		RCLCPP_ERROR( get_logger(), "Joint model group %s is not composed of single joints.", group_name.c_str() );
		return false;
	}

	return true;
}

// ------------------------------------------------------------

bool KinematicsSolver::CheckLimits( const std::span< const double >& joint_angles ) const
{
	assert( joint_model_ != nullptr );
	return joint_model_->satisfiesPositionBounds( & joint_angles[0] );
}

// ------------------------------------------------------------

} // namespace SOArm100::Kinematics
