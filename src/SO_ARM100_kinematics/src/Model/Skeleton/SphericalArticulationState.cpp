#include "Model/Skeleton/SphericalArticulationState.hpp"

#include "Global.hpp"
#include "Utils/Euler.hpp"

namespace SOArm100::Kinematics::Model
{

// ------------------------------------------------------------

SphericalArticulationState::SphericalArticulationState( ArticulationConstPtr articulation ) :
	ArticulationState( articulation )
{
	assert( articulation->GetType() == ArticulationType::Spherical );
	euler_configuration_ = Euler::ComputeConfiguration(
		articulation->Joints()[0]->Axis(),
		articulation->Joints()[1]->Axis(),
		articulation->Joints()[2]->Axis() );
}

// ------------------------------------------------------------

void SphericalArticulationState::ApplyConstraints( BoneState& bone_state ) const
{
	if ( bone_state.Direction().norm() != bone_state.GetBone()->Length() )
		bone_state.Direction() = bone_state.GetBone()->Length() * bone_state.Direction().normalized();
}

// ------------------------------------------------------------

void SphericalArticulationState::UpdateValues( const BoneState& bone_state )
{
	auto joint_0 = articulation_->Joints()[0];
	auto joint_1 = articulation_->Joints()[1];
	auto joint_2 = articulation_->Joints()[2];

	Vec3d old_bone = bone_state.GetBone()->Direction();
	Vec3d new_bone = world_transform_.rotation().inverse() * bone_state.Direction();
	
	Vec3d angles;
	if ( euler_configuration_.has_value() )
	{
		angles = Euler::Solve(
			*euler_configuration_, 
			joint_0->Axis(),
			joint_1->Axis(), 
			joint_2->Axis(), 
			old_bone, 
			new_bone );
	}

	SetJointInternalState( 
		joint_states_[0], 
		world_transform_, 
		global_transform_, 
		local_transform_, 
		angles[0] );

	SetJointInternalState( 
		joint_states_[1], 
		world_transform_, 
		global_transform_, 
		local_transform_, 
		angles[1] );

	SetJointInternalState( 
		joint_states_[2], 
		world_transform_, 
		global_transform_, 
		local_transform_, 
		angles[2] );
}

// ------------------------------------------------------------

}