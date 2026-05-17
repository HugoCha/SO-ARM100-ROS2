#include "Model/Skeleton/ArticulationState.hpp"

#include "Global.hpp"

#include "Model/Joint/JointState.hpp"
#include "Model/Skeleton/Articulation.hpp"

#include <cmath>
#include <memory>

namespace SOArm100::Kinematics::Model
{

// ------------------------------------------------------------

ArticulationState::ArticulationState( const Articulation* articulation ) :
	articulation_( articulation ),
	world_transform_( Iso3d::Identity() )
{
	world_transform_.translate( articulation->Center() );

	Iso3d local_transform = Iso3d::Identity();

	for ( auto it = articulation_->Joints().begin(); it != articulation->Joints().end(); ++it )
	{
		joint_states_.push_back( std::make_shared< JointState >( *it ) );
		SetJointInternalState(
			joint_states_.back(),
			local_transform,
			joint_states_.back()->Value(),
			1.0 );
	}

	local_transform_ = local_transform;
	global_transform_ = world_transform_ * local_transform_;
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
	const Iso3d& world_transform,
	const VecXd& values )
{
	world_transform_ = world_transform;

	Iso3d local_transform = Iso3d::Identity();

	for ( auto i = 0; i < joint_states_.size(); i++ )
	{
		SetJointInternalState(
			joint_states_[i],
			local_transform,
			values[i],
			1.0 );
	}

	local_transform_ = local_transform;
	global_transform_ = world_transform_ * local_transform_;
}

// ------------------------------------------------------------

void ArticulationState::SetJointInternalState(
	JointStatePtr& joint_state,
	Iso3d& internal_local_transform,
	double value,
	double damping_factor ) const
{
	auto joint = joint_state->GetJoint();
	auto internal_transform = world_transform_ * internal_local_transform;
	
	joint_state->Origin() =
		internal_transform.translation() +
		internal_transform.rotation() * ( joint->Origin() - articulation_->Center() );

	damping_factor = std::clamp( damping_factor, 0.1, 1.0 );

	joint_state->Axis() = internal_transform.rotation() * joint->Axis();

	double old_value = joint_state->Value();
	double new_value = old_value + damping_factor * ( value - old_value );
	joint_state->Value()  = joint->GetLimits().Clamp( new_value );

	if ( joint->IsPrismatic() )
	{
		Vec3d translation = joint_state->Value() * joint->Axis();
		internal_local_transform.translate( translation );
	}
	else if ( joint->IsRevolute() )
	{
		auto rotation = AngleAxis( joint_state->Value(), joint->Axis() );
		internal_local_transform.rotate( rotation );
	}
}

// ------------------------------------------------------------

void ArticulationState::SetCenterPose( const Iso3d& world_transform )
{
	world_transform_ = world_transform;

	Iso3d local_transform = Iso3d::Identity();

	for ( auto i = 0; i < joint_states_.size(); i++ )
	{
		SetJointInternalState(
			joint_states_[i],
			local_transform,
			joint_states_[i]->Value(),
			1.0 );
	}

	local_transform_ = local_transform;
	global_transform_ = world_transform_ * local_transform_;
}

// ------------------------------------------------------------

}