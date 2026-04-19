#pragma once

#include "Global.hpp"

#include "Model/Skeleton/Articulation.hpp"
#include "Model/Skeleton/Bone.hpp"

#include <cassert>

namespace SOArm100::Kinematics::Model
{
class ArticulationBoneConstraint
{
public:
ArticulationBoneConstraint( ArticulationConstPtr articulation, BoneConstPtr bone ) :
	articulation_( articulation ),
	bone_( bone )
{
	assert( articulation->Center() == bone->Origin() );
}

virtual void ApplyConstraint(
	const Quaternion& rotation,
	const Vec3d& center,
	Vec3d& direction ) const = 0;

protected:
ArticulationConstPtr articulation_;
BoneConstPtr bone_;
};
}