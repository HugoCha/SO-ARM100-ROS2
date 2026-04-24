#include "Model/Skeleton/RevoluteArticulationState.hpp"

#include "Global.hpp"

#include "Model/Skeleton/BoneState.hpp"
#include "Utils/MathUtils.hpp"

namespace SOArm100::Kinematics::Model
{

// ------------------------------------------------------------

RevoluteArticulationState::RevoluteArticulationState( ArticulationConstPtr articulation ) :
	ArticulationState( articulation )
{
	assert( articulation->GetType() == ArticulationType::Revolute );
}

// ------------------------------------------------------------

void RevoluteArticulationState::ApplyConstraints( BoneState& bone_state ) const
{
	auto joint = articulation_->Joints()[0];
	auto joint_state = joint_states_[0];
	auto bone = bone_state.GetBone();

	Vec3d v_ref = world_transform_.rotation() * bone->Direction();
	Vec3d v = bone_state.Direction();

	Vec3d v_ref_plane = v_ref - v_ref.dot( joint_state->Axis() ) * joint_state->Axis();
	Vec3d v_plane = v - v.dot( joint_state->Axis() ) * joint_state->Axis();

	if ( v_ref_plane.norm() < epsilon || v_plane.norm() < epsilon )
	{
		bone_state.Direction() = bone->Length() * joint_state->Axis();
		return;
	}

	double angle = SignedAngle( v_ref_plane, v_plane, joint_state->Axis() );
	angle = joint->GetLimits().Clamp( angle );
	auto rotation = AngleAxis( angle, joint_state->Axis() );

	bone_state.Direction() = rotation * v_ref;
}

// ------------------------------------------------------------

void RevoluteArticulationState::UpdateValues( const BoneState& bone_state )
{
	auto joint = articulation_->Joints()[0];
	auto joint_state = joint_states_[0];
	auto bone = bone_state.GetBone();

	Vec3d v_ref = world_transform_.rotation() * bone->Direction();
	Vec3d v = bone_state.Direction();

	Vec3d v_ref_plane = v_ref - v_ref.dot( joint_state->Axis() ) * joint_state->Axis();
	Vec3d v_plane = v - v.dot( joint_state->Axis() ) * joint_state->Axis();

	if ( v_ref_plane.norm() < epsilon || v_plane.norm() < epsilon )
	{
		return;
	}

	double angle = SignedAngle( v_ref_plane, v_plane, joint_state->Axis() );

	local_transform_.setIdentity();
	SetJointInternalState(
		joint_state,
		world_transform_,
		global_transform_,
		local_transform_,
		angle );
}

// ------------------------------------------------------------

}