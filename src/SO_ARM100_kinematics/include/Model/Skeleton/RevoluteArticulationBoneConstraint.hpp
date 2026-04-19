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
	const Quaternion& rotation,
	const Vec3d& center,
	Vec3d& direction ) const override;

private:
std::unique_ptr< const Base3d > base_ref_;
};
}