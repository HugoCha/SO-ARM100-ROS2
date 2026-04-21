#pragma once

#include "Global.hpp"

#include "ArticulationBoneConstraint.hpp"
#include "Model/Geometry/Line3d.hpp"

#include <memory>

namespace SOArm100::Kinematics::Model
{
class PrismaticArticulationBoneConstraint : public ArticulationBoneConstraint
{
public:
PrismaticArticulationBoneConstraint( ArticulationConstPtr articulation, BoneConstPtr bone );

virtual void ApplyConstraint(
	const Quaternion& articulation_rotation,
	const Vec3d& articulation_center,
	BoneState& bone_state ) const override;

private:
std::unique_ptr< const Line3d > line_ref_;
};
}