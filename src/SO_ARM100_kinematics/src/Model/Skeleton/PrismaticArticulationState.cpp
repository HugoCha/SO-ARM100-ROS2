#include "Model/Skeleton/PrismaticArticulationState.hpp"

#include "Global.hpp"

#include "Model/Skeleton/BoneState.hpp"
#include "Utils/MathUtils.hpp"

namespace SOArm100::Kinematics::Model
{

// ------------------------------------------------------------

PrismaticArticulationState::PrismaticArticulationState( const Articulation* articulation  ) :
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
	auto rotation     = world_transform_.rotation();

	Vec3d origin = world_transform_.translation();
	Vec3d axis = rotation * joint->Axis();

	// Slide
	Vec3d slide_axis = ProjectPointOnAxis( 
		bone_state.Origin() + bone_state.Direction(), 
		origin, 
		axis );

	// Compute Bone portion on slide axis
	Vec3d bone_on_axis = ProjectPointOnAxis( 
		origin + rotation * bone->Direction(), 
		origin, 
		axis );

	slide_axis -= bone_on_axis;
	double sign  = axis.dot( slide_axis ) >= 0 ? 1.0 : -1.0;
	double slide = sign >= 0 ? slide_axis.norm() : -slide_axis.norm();
	slide = joint->GetLimits().Clamp( slide );

	bone_state.Origin() = origin + slide * axis;
	bone_state.Direction() = rotation * bone->Direction();
}

// ------------------------------------------------------------

void PrismaticArticulationState::UpdateValues( 
	const BoneState& bone_state, 
	double damping_factor )
{
	auto joint = articulation_->Joints()[0];
	auto joint_state = joint_states_[0];
	auto bone = bone_state.GetBone();
	auto rotation = world_transform_.rotation();

	Vec3d origin = world_transform_.translation();
	Vec3d axis = rotation * joint->Axis();

	// Slide
	Vec3d slide_axis = ProjectPointOnAxis( 
		bone_state.Origin() + bone_state.Direction(), 
		origin, 
		axis );

	// Compute Bone portion on slide axis
	Vec3d bone_on_axis = ProjectPointOnAxis( 
		origin + rotation * bone->Direction(), 
		origin, 
		axis );

	slide_axis -= bone_on_axis;
	double sign  = axis.dot( slide_axis ) >= 0 ? 1.0 : -1.0;
	double slide = sign >= 0 ? slide_axis.norm() : -slide_axis.norm();
	slide = joint->GetLimits().Clamp( slide );

	Iso3d local_transform = Iso3d::Identity();

	SetJointInternalState(
		joint_state,
		local_transform,
		slide,
		damping_factor );

	local_transform_ = local_transform;
	global_transform_ = world_transform_ * local_transform_;
}

// ------------------------------------------------------------

}