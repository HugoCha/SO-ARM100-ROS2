#pragma once

#include "Global.hpp"

#include "ArticulationBoneConstraint.hpp"
#include "Model/Geometry/Base3d.hpp"

#include <memory>

namespace SOArm100::Kinematics::Model
{
class RevoluteArticulationBoneConstraint : public ArticulationBoneConstraint
{
public:
RevoluteArticulationBoneConstraint( ArticulationConstPtr articulation, BoneConstPtr bone );

virtual void ApplyConstraint(
	const Quaternion& articulation_rotation,
	const Vec3d& articulation_center,
	BoneState& bone_state ) const override;

private:
std::unique_ptr< const Base3d > base_ref_;
bool bone_on_axis_;
};
}