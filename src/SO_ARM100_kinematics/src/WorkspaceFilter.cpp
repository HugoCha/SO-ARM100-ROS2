#include "WorkspaceFilter.hpp"

#include "Converter.hpp"
#include "KinematicsUtils.hpp"

#include <Eigen/src/Geometry/Transform.h>
#include <cassert>
#include <moveit/robot_model/fixed_joint_model.hpp>
#include <moveit/robot_model/joint_model.hpp>
#include <moveit/robot_model/link_model.hpp>
#include <moveit/robot_model/prismatic_joint_model.hpp>
#include <moveit/robot_model/revolute_joint_model.hpp>
#include <moveit/robot_model/robot_model.hpp>

namespace SOArm100::Kinematics
{

// ------------------------------------------------------------

WorkspaceFilter::WorkspaceFilter( 
    const std::span< const moveit::core::JointModel* const >& joint_models ) :
    joint_models_( joint_models )
{
}

// ------------------------------------------------------------

const Mat4d WorkspaceFilter::GetBaseFrame() const
{
	assert(!joint_models_.empty());
    const auto* base_link = joint_models_[0]->getParentLinkModel();
    return base_link->getJointOriginTransform().matrix();
}

// ------------------------------------------------------------

bool WorkspaceFilter::IsUnreachable( const geometry_msgs::msg::Pose& target_pose ) const
{
	const auto& target_frame = ToMat4d( target_pose );
	double distance = ( Translation( target_frame ) - Translation( GetBaseFrame() ) ).norm();
	return distance < min_reach_ || distance > max_reach_;
}

// ------------------------------------------------------------

void WorkspaceFilter::ComputeWorkspace(const std::span< const moveit::core::JointModel* const >& joint_models)
{
	min_reach_ = ComputeMinReach( joint_models );
	max_reach_ = ComputeMaxReach( joint_models );
}

// ------------------------------------------------------------

double WorkspaceFilter::ComputeMinReach(const std::span< const moveit::core::JointModel* const >&joint_models) const
{
	return 0.0;
}

// ------------------------------------------------------------

double WorkspaceFilter::ComputeMaxReach(const std::span< const moveit::core::JointModel* const >& joint_models) const
{
	double max_reach = 0.0;

	for ( const auto* joint : joint_models )
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