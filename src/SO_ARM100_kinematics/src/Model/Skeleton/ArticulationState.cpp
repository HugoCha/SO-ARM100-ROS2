#include "Model/Skeleton/ArticulationState.hpp"

#include "Global.hpp"

#include "Model/Joint/JointState.hpp"
#include "Model/Skeleton/Articulation.hpp"

#include <cmath>
#include <memory>

namespace SOArm100::Kinematics::Model
{

// ------------------------------------------------------------

ArticulationState::ArticulationState( ArticulationConstPtr articulation ) :
	articulation_( articulation ),
	center_( articulation->Center() ),
	rotation_( Quaternion::Identity() ),
	internal_transform_( Iso3d::Identity() )
{
	for ( auto it = articulation_->Joints().begin(); it != articulation->Joints().end(); ++it )
	{
		joint_states_.push_back( std::make_shared< JointState >( *it ) );
	}
}

// ------------------------------------------------------------

std::vector< JointStateConstPtr > ArticulationState::GetJointStates() const
{
	std::vector< JointStateConstPtr > joint_states;
	joint_states.reserve( joint_states_.size() );

	for ( const auto& state : joint_states_ )
		joint_states.push_back( state );

	return joint_states;
}

// ------------------------------------------------------------

VecXd ArticulationState::GetJointValues() const
{
	VecXd joints( articulation_->JointCount() );

	for ( auto i = 0; i < joint_states_.size(); i++ )
		joints[i] = joint_states_[i]->Value();

	return joints;
}

// ------------------------------------------------------------

void ArticulationState::SetState(
	const Quaternion& rotation,
	const Vec3d& origin,
	const VecXd& values )
{
	center_ = origin;
	rotation_ = rotation;

	Vec3d angle_axes;
	Quaternion internal_rotation = Quaternion::Identity();
	Quaternion intermediate_rotation = rotation;
	for ( auto i = 0; i < joint_states_.size(); i++ )
	{
		auto joint = articulation_->Joints()[i];
		auto joint_state = joint_states_[i];
		joint_state->Origin() = origin + intermediate_rotation * ( joint->Origin() - articulation_->Center() );
		joint_state->Axis()   = intermediate_rotation * joint->Axis();
		joint_state->Value()  = joint->GetLimits().Clamp( values[i] );
		angle_axes += joint_state->Value() * joint_state->Axis(); 
		internal_rotation = internal_rotation * AngleAxis( joint_state->Value(), joint->Axis() );
		intermediate_rotation = rotation * internal_rotation;
	}
	internal_rotation_ = internal_rotation;
	value_ = angle_axes.norm();
	axis_  = angle_axes / angle_axes.norm();
}

// ------------------------------------------------------------

void ArticulationState::SetCenterPose( const Quaternion& rotation, const Vec3d& origin )
{
	center_ = origin;
	rotation_ = rotation;

	Vec3d angle_axes;
	Quaternion internal_rotation = Quaternion::Identity();
	Quaternion intermediate_rotation = rotation;
	for ( auto i = 0; i < joint_states_.size(); i++ )
	{
		auto joint = articulation_->Joints()[i];
		auto joint_state = joint_states_[i];
		joint_state->Origin() = origin + intermediate_rotation * ( joint->Origin() - articulation_->Center() );
		joint_state->Axis()   = intermediate_rotation * joint->Axis();
		angle_axes += joint_state->Value() * joint_state->Axis(); 
		internal_rotation = internal_rotation * AngleAxis( joint_state->Value(), joint->Axis() );
		intermediate_rotation = rotation * internal_rotation;
	}
	internal_rotation_ = internal_rotation;
	value_ = angle_axes.norm();
	axis_  = angle_axes / angle_axes.norm();
}

// ------------------------------------------------------------

}