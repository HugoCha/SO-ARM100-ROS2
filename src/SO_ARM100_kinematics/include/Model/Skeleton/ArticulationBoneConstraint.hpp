#pragma once

#include "Global.hpp"

#include "Model/Skeleton/Articulation.hpp"
#include "Model/Skeleton/Bone.hpp"

#include <cassert>
#include <memory>

namespace SOArm100::Kinematics::Model
{
class BoneState;

class ArticulationBoneConstraint
{
public:
ArticulationBoneConstraint( ArticulationConstPtr articulation, BoneConstPtr bone ) :
	articulation_( articulation ),
	bone_( bone )
{
	assert( articulation->Center() == bone->Origin() );
}
virtual ~ArticulationBoneConstraint() = default;

virtual void ApplyConstraint(
	const Quaternion& articulation_rotation,
	const Vec3d& articulation_center,
	BoneState& bone_state ) const = 0;

protected:
ArticulationConstPtr articulation_;
BoneConstPtr bone_;
};

using ArticulationBoneConstraintUniqueConstPtr = std::unique_ptr< const ArticulationBoneConstraint >;
}