#include "WorkspaceFilter.hpp"

#include "Converter.hpp"
#include "KinematicsUtils.hpp"
#include "Types.hpp"

#include <Eigen/src/Geometry/Transform.h>
#include <moveit/robot_model/fixed_joint_model.hpp>
#include <moveit/robot_model/fixed_joint_model.hpp>
#include <moveit/robot_model/joint_model.hpp>
#include <moveit/robot_model/joint_model_group.hpp>
#include <moveit/robot_model/link_model.hpp>
#include <moveit/robot_model/prismatic_joint_model.hpp>
#include <moveit/robot_model/revolute_joint_model.hpp>
#include <moveit/robot_model/robot_model.hpp>

namespace SOArm100::Kinematics
{

// ------------------------------------------------------------

WorkspaceFilter::WorkspaceFilter(
	const moveit::core::RobotModelConstPtr& robot_model,
	const std::string& group_name,
	const std::string& base_frame_name ) :
	base_frame_name_( base_frame_name )
{
	if ( !robot_model )
		return;

	joints_ = robot_model->getJointModelGroup( group_name );
	ComputeWorkspace( joints_, base_frame_name );
}

// ------------------------------------------------------------

bool WorkspaceFilter::IsUnreachable( const geometry_msgs::msg::Pose& target_pose ) const
{
	const auto& base_frame = GetBaseFrame();
	const auto& target_frame = ToMat4d( target_pose );
	double distance = ( Translation( target_frame ) - Translation( base_frame ) ).norm();
	return distance < min_reach_ || distance > max_reach_;
}

// ------------------------------------------------------------

const Mat4d WorkspaceFilter::GetBaseFrame() const
{
	const auto* base_link = joints_->getLinkModel( base_frame_name_ );

	if ( !base_link )
	{
		return Mat4d::Identity();
	}

	return base_link->getJointOriginTransform().matrix();
}

// ------------------------------------------------------------

void WorkspaceFilter::ComputeWorkspace(
	const moveit::core::JointModelGroup* joint_model,
	const std::string& base_frame_name )
{
	min_reach_ = ComputeMinReach( joints_, base_frame_name );
	max_reach_ = ComputeMaxReach( joints_, base_frame_name );
}

// ------------------------------------------------------------

double WorkspaceFilter::ComputeMinReach(
	const moveit::core::JointModelGroup* joints,
	const std::string& base_frame_name ) const
{
	return 0.0;
}

// ------------------------------------------------------------

double WorkspaceFilter::ComputeMaxReach(
	const moveit::core::JointModelGroup* joints,
	const std::string& base_frame_name ) const
{
	double max_reach = 0.0;

	for ( const auto* joint : joints->getJointModels() )
	{
		switch ( joint->getType() )
		{
		case moveit::core::JointModel::JointType::FIXED:
		{
			const auto* fixed_joint = static_cast< const moveit::core::FixedJointModel* >( joint );
			const auto* link = fixed_joint->getChildLinkModel();
			const auto& local_origin = link->getJointOriginTransform();
			max_reach += local_origin.translation().norm();
			break;
		}
		case moveit::core::JointModel::JointType::PRISMATIC:
		{
			const auto* prismatic_joint = static_cast< const moveit::core::PrismaticJointModel* >( joint );
			const auto* link = prismatic_joint->getChildLinkModel();
			const auto& axis = prismatic_joint->getAxis().normalized();
			const auto& local_origin = link->getJointOriginTransform();
			const auto& max_extension = joint->getMaximumExtent() * axis;
			max_reach += local_origin.translation().norm() + max_extension.norm();
			break;
		}
		case moveit::core::JointModel::JointType::REVOLUTE:
		{
			const auto* revolute_joint = static_cast< const moveit::core::RevoluteJointModel* >( joint );
			const auto* link = revolute_joint->getChildLinkModel();
			const auto& local_origin = link->getJointOriginTransform();
			max_reach += local_origin.translation().norm();
			break;
		}
		default:
		{
			assert( "Only single DOF joint are supported" );
			break;
		}
		}
	}
	return max_reach;
}

// ------------------------------------------------------------

}