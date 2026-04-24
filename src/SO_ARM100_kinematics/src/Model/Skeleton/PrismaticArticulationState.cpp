#include "Model/Skeleton/PrismaticArticulationState.hpp"

#include "Global.hpp"

#include "Model/Skeleton/BoneState.hpp"
#include "Utils/MathUtils.hpp"

namespace SOArm100::Kinematics::Model
{

// ------------------------------------------------------------

PrismaticArticulationState::PrismaticArticulationState( ArticulationConstPtr articulation  ) :
	ArticulationState( articulation )
{
	assert( articulation->GetType() == ArticulationType::Prismatic );
}

// ------------------------------------------------------------

void PrismaticArticulationState::ApplyConstraints( BoneState& bone_state ) const
{
	auto joint = articulation_->Joints()[0];
	auto joint_state = joint_states_[0];
	auto bone = bone_state.GetBone();

	Vec3d translation = world_transform_.translation();
	auto rotation     = world_transform_.rotation();
	Vec3d axis = joint_state->Axis();

	Vec3d bone_end = bone_state.Origin() + bone_state.Direction();
	Vec3d slide_axis = ProjectPointOnAxis( bone_end, translation, axis );

	Vec3d current_bone_dir = world_transform_.rotation() * bone->Direction();
	Vec3d current_bone_end = world_transform_.translation() + current_bone_dir;
	Vec3d bone_on_axis = ProjectPointOnAxis( current_bone_end, translation, axis );

	slide_axis -= bone_on_axis;
	double sign  = axis.dot( slide_axis ) >= 0 ? 1.0 : -1.0;
	double slide = sign >= 0 ? slide_axis.norm() : -slide_axis.norm();
	slide = joint->GetLimits().Clamp( slide );

	bone_state.Origin() = translation;
	bone_state.Direction() = slide * axis + current_bone_dir;
}

// ------------------------------------------------------------

void PrismaticArticulationState::UpdateValues( const BoneState& bone_state )
{
	auto joint = articulation_->Joints()[0];
	auto joint_state = joint_states_[0];
	auto bone = bone_state.GetBone();

	Vec3d origin = joint_state->Origin();
	Vec3d axis = joint_state->Axis();

	Vec3d bone_end = bone_state.Origin() + bone_state.Direction();
	Vec3d slide_axis = ProjectPointOnAxis( bone_end, origin, axis );

	Vec3d current_bone_dir = world_transform_.rotation() * bone->Direction();
	Vec3d current_bone_end = world_transform_.translation() + current_bone_dir;
	Vec3d bone_on_axis = ProjectPointOnAxis( current_bone_end, origin, axis );

	slide_axis -= bone_on_axis;
	double sign  = axis.dot( slide_axis ) >= 0 ? 1.0 : -1.0;
	double slide = sign >= 0 ? slide_axis.norm() : -slide_axis.norm();
	slide = joint->GetLimits().Clamp( slide );

	local_transform_.setIdentity();

	SetJointInternalState(
		joint_state,
		world_transform_,
		global_transform_,
		local_transform_,
		slide );
}

// ------------------------------------------------------------

}