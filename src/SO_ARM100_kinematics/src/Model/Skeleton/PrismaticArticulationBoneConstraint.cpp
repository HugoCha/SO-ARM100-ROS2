#include "Model/Skeleton/PrismaticArticulationBoneConstraint.hpp"

#include "Global.hpp"

#include "Model/Geometry/Line3d.hpp"
#include "Model/Skeleton/BoneState.hpp"

#include <memory>

namespace SOArm100::Kinematics::Model
{

// ------------------------------------------------------------

PrismaticArticulationBoneConstraint::PrismaticArticulationBoneConstraint(
	ArticulationConstPtr articulation,
	BoneConstPtr bone ) :
	ArticulationBoneConstraint( articulation, bone )
{
	assert( articulation->GetType() == ArticulationType::Prismatic );

	auto joint = articulation_->Joints()[0];
	line_ref_ = std::make_unique< const Line3d >( joint->Origin(), joint->Axis() );
}

// ------------------------------------------------------------

void PrismaticArticulationBoneConstraint::ApplyConstraint(
	const Quaternion& articulation_rotation,
	const Vec3d& articulation_center,
	BoneState& bone_state ) const
{
	auto joint = articulation_->Joints()[0];
	auto transformed_line = Transform( *line_ref_, articulation_center, articulation_rotation );

	Vec3d bone_end = bone_state.Origin() + bone_state.Direction();
	Vec3d slide_axis = ProjectPoint( transformed_line, bone_end );

	Vec3d current_bone_dir = articulation_rotation * bone_->Direction();
	Vec3d current_bone_end = transformed_line.point + current_bone_dir;
	Vec3d bone_on_axis = ProjectPoint( transformed_line, current_bone_end );
	Vec3d bone_off_axis = current_bone_dir - bone_on_axis;

	slide_axis -= bone_on_axis;
	double sign  = transformed_line.axis.dot( slide_axis ) >= 0 ? 1.0 : -1.0;
	double slide = sign >= 0 ? slide_axis.norm() : -slide_axis.norm();
	slide = joint->GetLimits().Clamp( slide );

	bone_state.Origin() = transformed_line.point;
	bone_state.Direction() =
		slide * transformed_line.axis +
		current_bone_dir;
}

// ------------------------------------------------------------

}