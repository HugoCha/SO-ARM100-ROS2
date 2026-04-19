#pragma once

#include "Global.hpp"

#include "ArticulationBoneConstraint.hpp"

namespace SOArm100::Kinematics::Model
{
class PrismaticArticulationBoneConstraint : public ArticulationBoneConstraint
{
public:
PrismaticArticulationBoneConstraint( ArticulationConstPtr articulation, BoneConstPtr bone );

virtual void ApplyConstraint(
	const Quaternion& rotation,
	const Vec3d& center,
	Vec3d& direction ) const override;
};

}