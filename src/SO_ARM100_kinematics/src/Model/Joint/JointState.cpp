#include "Model/Joint/JointState.hpp"

#include "Global.hpp"

#include "Model/Geometry/Pose.hpp"
#include "Model/Joint/Joint.hpp"
#include "Utils/MathUtils.hpp"

namespace SOArm100::Kinematics::Model
{

// ------------------------------------------------------------

JointState::JointState( const JointConstPtr& joint ) :
	joint_( joint ),
	pose_( { joint->Origin(), joint->Axis() } ),
	value_( joint->GetLimits().Within( 0 ) ? 0 : joint->GetLimits().Center() ),
	update_value_required_( false ),
	update_pose_required_( false )
{
}

// ------------------------------------------------------------

void JointState::SetState( const Vec3d& origin, const Vec3d& axis, double value )
{
	pose_.origin = origin;
	pose_.axis   = axis;
	value_       = value;
}

// ------------------------------------------------------------

void JointState::SetPose( const Vec3d& origin, const Vec3d& axis )
{
	pose_.origin = origin;
	pose_.axis   = axis;
	update_value_required_ = true;
	update_pose_required_ = false;
}

// ------------------------------------------------------------

void JointState::UpdatePose( const Vec3d& origin, const Vec3d& axis )
{
	pose_.origin = origin;
	pose_.axis   = axis;
	update_pose_required_ = false;
}

// ------------------------------------------------------------

void JointState::UpdateValue(
	const Vec3d& old_orgin,
	const Vec3d& old_to_child_direction,
	const Vec3d& new_to_child_direction )
{
	if ( joint_->IsPrismatic() )
	{
		UpdatePrismaticValue( old_orgin );
	}
	else if ( joint_->IsRevolute() )
	{
		UpdateRevoluteValue(
			old_to_child_direction,
			new_to_child_direction );
	}
	update_value_required_ = false;
}

// ------------------------------------------------------------

void JointState::UpdatePrismaticValue(
	const Vec3d& old_orgin )
{
	Vec3d direction = pose_.origin - old_orgin;
	double delta = direction.dot( pose_.axis );
	double value = value_ + delta;
	if ( !joint_->GetLimits().Within( value ) )
	{
		value = joint_->GetLimits().Clamp( value );
		update_pose_required_ = true;
	}
	value_ = value;
}

// ------------------------------------------------------------

void JointState::UpdateRevoluteValue(
	const Vec3d& old_to_child_direction,
	const Vec3d& new_to_child_direction )
{
	Vec3d old_proj = ProjectVectorOnPlane( old_to_child_direction, pose_.axis );
	Vec3d new_proj = ProjectVectorOnPlane( new_to_child_direction, pose_.axis );

	if ( old_proj.norm() < epsilon || new_proj.norm() < epsilon )
		return;

	double value = value_ + SignedAngle( old_proj, new_proj, pose_.axis );

	if ( !joint_->GetLimits().Within( value ) )
	{
		value = joint_->GetLimits().Clamp( value );
		update_pose_required_ = true;
	}
	value_ = value;
}

// ------------------------------------------------------------

}