#include "WorkspaceFilter.hpp"

#include "Joint/JointChain.hpp"
#include "Utils/Converter.hpp"
#include "Utils/KinematicsUtils.hpp"

#include <cassert>

namespace SOArm100::Kinematics
{

// ------------------------------------------------------------

WorkspaceFilter::WorkspaceFilter( const JointChain& joint_chain )
{
	base_frame_ = joint_chain.GetJoints()[0]->GetTwist().GetLinear();
	ComputeWorkspace( joint_chain );
}

// ------------------------------------------------------------

const Vec3d WorkspaceFilter::GetBaseFrame() const
{
	return base_frame_;
}

// ------------------------------------------------------------

bool WorkspaceFilter::IsUnreachable( const geometry_msgs::msg::Pose& target_pose ) const
{
	return IsUnreachable( ToMat4d( target_pose ) );
}

// ------------------------------------------------------------

bool WorkspaceFilter::IsUnreachable( const Mat4d& target_pose ) const
{
	double distance = ( Translation( target_pose ) - GetBaseFrame() ).norm();
	return distance < min_reach_ || distance > max_reach_;
}

// ------------------------------------------------------------

void WorkspaceFilter::ComputeWorkspace( const JointChain& joint_chain )
{
	min_reach_ = ComputeMinReach( joint_chain );
	max_reach_ = ComputeMaxReach( joint_chain );
}

// ------------------------------------------------------------

double WorkspaceFilter::ComputeMinReach( const JointChain& joint_chain ) const
{
	return 0.0;
}

// ------------------------------------------------------------

double WorkspaceFilter::ComputeMaxReach( const JointChain& joint_chain ) const
{
	double max_reach = 0.0;

	for ( const auto joint : joint_chain.GetJoints() )
	{
		switch ( joint->GetType() )
		{
		case JointType::FIXED:
		case JointType::REVOLUTE:
		{
			max_reach += joint->GetLink().GetLength();
			break;
		}
		case JointType::PRISMATIC:
		{
			max_reach += joint->GetLink().GetLength() + joint->GetLimits().Max();
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