#include "KinematicsSolver.hpp"

#include "Converter.hpp"
#include "KinematicsUtils.hpp"
#include "Twist.hpp"
#include "WorkspaceFilter.hpp"

#include <cassert>
#include <memory>
#include <moveit/robot_model/joint_model.hpp>
#include <moveit/robot_model/joint_model_group.hpp>
#include <moveit/robot_model/link_model.hpp>
#include <moveit/robot_model/prismatic_joint_model.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <rclcpp/logger.hpp>
#include <vector>

namespace SOArm100::Kinematics
{

// ------------------------------------------------------------

static rclcpp::Logger get_logger()
{
	static rclcpp::Logger logger = rclcpp::get_logger( "KinematicsSolver" );
	return logger;
}

// ------------------------------------------------------------

KinematicsSolver::KinematicsSolver() :
	twists_(),
	p_home_configuration_( std::make_unique< const Mat4d >( Mat4d::Identity() ) )
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
	const auto* joint_model_group = robot_model->getJointModelGroup( group_name );
	
	const auto& active_joint_models = joint_model_group->getActiveJointModels();
	joint_models_ = std::span< const moveit::core::JointModel* const >( active_joint_models );
	
	p_workspace_filter_ = std::make_unique< WorkspaceFilter >( joint_models_ );

	const Mat4d& home_configuration = state.getGlobalLinkTransform( tip_frames[0] ).matrix();
	p_home_configuration_ = std::make_unique< const Mat4d >( home_configuration );

	auto twists = std::vector< TwistConstPtr >( joint_models_.size() );

	for ( const auto* joint_model : joint_models_ )
	{
		Vec3d axis;
		if ( joint_model->getType() == moveit::core::JointModel::REVOLUTE )
		{
			const auto* revolute_joint_model =
				static_cast< const moveit::core::RevoluteJointModel* >( joint_model );
			axis = revolute_joint_model->getAxis();
		}
		else if ( joint_model->getType() == moveit::core::JointModel::PRISMATIC )
		{
			const auto* prismatic_joint_model =
				static_cast< const moveit::core::PrismaticJointModel* >( joint_model );
			axis = prismatic_joint_model->getAxis();
		}

		const auto& joint_transform =
			state.getGlobalLinkTransform( joint_model->getChildLinkModel() );

		const Vec3d axis_world = joint_transform.rotation() * axis;
		const Vec3d point_on_axis_world = joint_transform.translation();
		const auto& bound = joint_model->getVariableBounds()[0];

		auto twist = std::make_shared< const Twist >( 
			axis_world, 
			point_on_axis_world,
			bound.min_position_, 
			bound.max_position_ );

		twists.emplace_back( twist );
	}
}

// ------------------------------------------------------------

void KinematicsSolver::Initialize(
	const std::span< const moveit::core::JointModel* const >& joint_models,
	const std::span< TwistConstPtr >& twists,
	const Mat4d& home_configuration,
	double search_discretization )
{
	joint_models_ = joint_models;
	p_workspace_filter_ = std::make_unique< WorkspaceFilter >( joint_models );
	p_home_configuration_ = std::make_unique< const Mat4d >( home_configuration );
	twists_ = twists;
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
	if ( !joint_models_.empty() )
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

	POE( twists_, *p_home_configuration_, joint_angles, pose );

	return true;
}

// ------------------------------------------------------------

bool KinematicsSolver::InverseKinematic(
	const geometry_msgs::msg::Pose& target_pose,
	const std::span< const double >& seed_joints,
	std::vector< double >& joints ) const
{
	joints.clear();

	if ( p_workspace_filter_->IsUnreachable( target_pose ) )
	{
		return false;
	}

	const auto& target = ToMat4d( target_pose );
	VecXd output_joints;

	if ( InverseKinematic( target, seed_joints, output_joints ) )
	{
		joints = ToStdVector( output_joints );
		return true;
	}

	return false;
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
	assert( twists_.size() == joint_angles.size() );
	for ( auto i = 0; i < twists_.size(); i++ )
		if ( !twists_[i]->Limits().SatisfyLimits( joint_angles[i] ) )
			return false;
	return true;
}

// ------------------------------------------------------------

} // namespace SOArm100::Kinematics
