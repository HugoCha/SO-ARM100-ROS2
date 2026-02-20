#include "KinematicsSolver.hpp"

#include "Joint/JointChain.hpp"
#include "Joint/Twist.hpp"
#include "Utils/Converter.hpp"
#include "Utils/KinematicsUtils.hpp"
#include "WorkspaceFilter.hpp"

#include <cassert>
#include <memory>
#include <moveit/robot_model/fixed_joint_model.hpp>
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
	joint_chain_( nullptr ),
	home_configuration_( std::make_unique< const Mat4d >( Mat4d::Identity() ) )
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

	const auto& joint_models = joint_model_group->getJointModels();

	const Mat4d& home_configuration = state.getGlobalLinkTransform( tip_frames[0] ).matrix();
	home_configuration_ = std::make_shared< const Mat4d >( home_configuration );

	auto joint_chain = std::make_shared< JointChain >( joint_models.size() );

	for ( const auto* joint_model : joint_models )
	{
		const auto& joint_transform =
			state.getGlobalLinkTransform( joint_model->getChildLinkModel() );
		Link link( joint_transform.matrix() );

		Limits limits;
		const auto& bounds = joint_model->getVariableBounds();
		if ( !bounds.empty() )
		{
			const auto& lim = bounds[0];
			limits = Limits( lim.min_position_, lim.max_position_ );
		}

		Twist twist;
		if ( joint_model->getType() == moveit::core::JointModel::REVOLUTE )
		{
			const auto* revolute_joint_model =
				static_cast< const moveit::core::RevoluteJointModel* >( joint_model );

			const Vec3d axis_world = joint_transform.rotation() * revolute_joint_model->getAxis();
			const Vec3d point_on_axis_world = joint_transform.translation();

			twist = Twist( axis_world, point_on_axis_world );
		}
		else if ( joint_model->getType() == moveit::core::JointModel::PRISMATIC )
		{
			const auto* prismatic_joint_model =
				static_cast< const moveit::core::PrismaticJointModel* >( joint_model );

			twist = Twist( prismatic_joint_model->getAxis() );
		}

		joint_chain->Add( twist, link, limits );
	}
	joint_chain_ = std::move( joint_chain );
	workspace_filter_ = std::make_unique< WorkspaceFilter >( *joint_chain_ );
}

// ------------------------------------------------------------

void KinematicsSolver::Initialize(
	std::shared_ptr< const JointChain > joint_chain,
	std::shared_ptr< const Mat4d > home_configuration,
	double search_discretization )
{
	joint_chain_ = joint_chain;
	home_configuration_ = home_configuration;
	workspace_filter_ = std::make_unique< WorkspaceFilter >( *joint_chain );
}

// ------------------------------------------------------------

bool KinematicsSolver::ForwardKinematic(
	const std::span< const double >& joints,
	geometry_msgs::msg::Pose& pose ) const
{
	Mat4d T_end;
	if ( ForwardKinematic( ToVecXd( joints ), T_end ) )
	{
		pose = ToPoseMsg( T_end );
		return true;
	}
	return false;
}

// ------------------------------------------------------------

bool KinematicsSolver::ForwardKinematic(
	const VecXd& joints,
	Mat4d& pose ) const
{
	if ( !joint_chain_  )
	{
		RCLCPP_ERROR( get_logger(), "Joint model not initialized." );
		return false;
	}

	if ( joints.size() != joint_chain_->GetActiveJointCount() )
	{
		RCLCPP_ERROR(
			get_logger(),
			"Joint angles size (%zu) does not match number of twists (%zu).",
			joints.size(), joint_chain_->GetActiveJointCount() );
		return false;
	}

	POE( *joint_chain_, *home_configuration_, joints, pose );

	return true;
}

// ------------------------------------------------------------

bool KinematicsSolver::InverseKinematic(
	const geometry_msgs::msg::Pose& target_pose,
	const std::span< const double >& seed_joints,
	std::vector< double >& joints ) const
{
	if ( joints.size() != joint_chain_->GetActiveJointCount() )
		joints.resize( joint_chain_->GetActiveJointCount() );

	return InverseKinematicImpl( 
		ToTransformMatrix( target_pose ),
		seed_joints, 
		joints.data() );
}

// ------------------------------------------------------------

bool KinematicsSolver::InverseKinematic(
	const Mat4d& target_pose,
	const std::span< const double >& seed_joints,
	VecXd& joints ) const
{
	if ( joints.size() != joint_chain_->GetActiveJointCount() )
		joints.resize( joint_chain_->GetActiveJointCount() );

	return InverseKinematicImpl( 
		target_pose, 
		seed_joints, 
		joints.data() );
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

bool KinematicsSolver::IsUnreachable( const Mat4d& target ) const
{
	assert( workspace_filter_ );
	return workspace_filter_->IsUnreachable( target );
}

// ------------------------------------------------------------

bool KinematicsSolver::CheckLimits( const std::span< const double >& joints ) const
{
	assert( joint_chain_->GetActiveJointCount() == joints.size() );

	const auto& active_joints = joint_chain_->GetActiveJoints();
	for ( size_t i = 0; i < joint_chain_->GetActiveJointCount(); i++ )
		if ( !active_joints[i]->GetLimits().Within( joints[i] ) )
			return false;
	return true;
}

// ------------------------------------------------------------

} // namespace SOArm100::Kinematics
