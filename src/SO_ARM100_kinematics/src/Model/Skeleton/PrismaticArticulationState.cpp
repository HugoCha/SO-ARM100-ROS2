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

	Vec3d origin = joint_state->Origin();
	Vec3d axis = joint_state->Axis();

	Vec3d bone_end = bone_state.Origin() + bone_state.Direction();
	Vec3d slide_axis = ProjectPointOnAxis( bone_end, origin, axis );

	Vec3d current_bone_dir = rotation_ * bone->Direction();
	Vec3d current_bone_end = origin + current_bone_dir;
	Vec3d bone_on_axis = ProjectPointOnAxis( current_bone_end, origin, axis );
	Vec3d bone_off_axis = current_bone_dir - bone_on_axis;

	slide_axis -= bone_on_axis;
	double sign  = axis.dot( slide_axis ) >= 0 ? 1.0 : -1.0;
	double slide = sign >= 0 ? slide_axis.norm() : -slide_axis.norm();
	slide = joint->GetLimits().Clamp( slide );

	bone_state.Origin() = origin;
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

	Vec3d current_bone_dir = rotation_ * bone->Direction();
	Vec3d current_bone_end = origin + current_bone_dir;
	Vec3d bone_on_axis = ProjectPointOnAxis( current_bone_end, origin, axis );
	Vec3d bone_off_axis = current_bone_dir - bone_on_axis;

	slide_axis -= bone_on_axis;
	double sign  = axis.dot( slide_axis ) >= 0 ? 1.0 : -1.0;
	double slide = sign >= 0 ? slide_axis.norm() : -slide_axis.norm();
	
	joint_state->Value() = joint->GetLimits().Clamp( slide );
}

// ------------------------------------------------------------

}