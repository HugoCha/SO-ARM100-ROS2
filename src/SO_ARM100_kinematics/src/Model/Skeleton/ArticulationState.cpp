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
	world_transform_( Iso3d::Identity() )
{
	world_transform_.translate( articulation->Center() );

	Iso3d global_transform = world_transform_;
	Iso3d local_transform = Iso3d::Identity();
	for ( auto it = articulation_->Joints().begin(); it != articulation->Joints().end(); ++it )
	{
		joint_states_.push_back( std::make_shared< JointState >( *it ) );
		SetJointInternalState(
			joint_states_.back(),
			world_transform_,
			global_transform,
			local_transform,
			joint_states_.back()->Value() );
	}

	global_transform_ = global_transform;
	local_transform_ = local_transform;
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

	Iso3d global_transform = world_transform_;
	Iso3d local_transform = Iso3d::Identity();

	for ( auto i = 0; i < joint_states_.size(); i++ )
	{
		SetJointInternalState(
			joint_states_[i],
			world_transform,
			global_transform,
			local_transform,
			values[i] );
	}

	global_transform_ = global_transform;
	local_transform_ = local_transform;
}

// ------------------------------------------------------------

void ArticulationState::SetJointInternalState(
	JointStatePtr& joint_state,
	const Iso3d& world_transform,
	Iso3d& global_transform,
	Iso3d& local_transform,
	double value ) const
{
	auto joint = joint_state->GetJoint();
	joint_state->Origin() =
		global_transform.translation() +
		global_transform.rotation() * ( joint->Origin() - articulation_->Center() );
	joint_state->Axis()   = global_transform.rotation() * joint->Axis();
	joint_state->Value()  = joint->GetLimits().Clamp( value );

	if ( joint->IsPrismatic() )
	{
		Vec3d translation = joint_state->Value() * joint->Axis();
		local_transform.translate( translation );
	}
	else if ( joint->IsRevolute() )
	{
		auto rotation = AngleAxis( joint_state->Value(), joint->Axis() );
		local_transform.rotate( rotation );
	}
	global_transform = world_transform * local_transform;
}

// ------------------------------------------------------------

void ArticulationState::SetCenterPose( const Iso3d& world_transform )
{
	world_transform_ = world_transform;

	Iso3d global_transform = world_transform_;
	Iso3d local_transform = Iso3d::Identity();

	for ( auto i = 0; i < joint_states_.size(); i++ )
	{
		SetJointInternalState(
			joint_states_[i],
			world_transform,
			global_transform,
			local_transform,
			joint_states_[i]->Value() );
	}

	global_transform_ = global_transform;
	local_transform_ = local_transform;
}

// ------------------------------------------------------------

}