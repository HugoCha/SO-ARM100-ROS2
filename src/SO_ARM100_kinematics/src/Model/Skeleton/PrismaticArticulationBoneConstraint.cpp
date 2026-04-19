#include "Model/Skeleton/PrismaticArticulationBoneConstraint.hpp"
#include "Global.hpp"

namespace SOArm100::Kinematics::Model
{

// ------------------------------------------------------------

PrismaticArticulationBoneConstraint::PrismaticArticulationBoneConstraint(
	ArticulationConstPtr articulation,
	BoneConstPtr bone ) :
	ArticulationBoneConstraint( articulation, bone )
{
	assert( articulation->GetType() == ArticulationType::Prismatic );
	assert( articulation->Axis() == bone->Direction() );
}

// ------------------------------------------------------------

void PrismaticArticulationBoneConstraint::ApplyConstraint(
	const Quaternion& rotation,
	const Vec3d& center,
	Vec3d& direction ) const
{
	Vec3d axis = rotation * articulation_->Axis();
	double slide = axis.dot( direction );
	auto joint = articulation_->Joints()[0];
	slide = joint->GetLimits().Clamp( slide );
	direction = slide * axis;
}

// ------------------------------------------------------------

}