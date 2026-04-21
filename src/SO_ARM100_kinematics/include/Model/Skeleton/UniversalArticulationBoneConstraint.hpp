#pragma once

#include "Global.hpp"

#include "ArticulationBoneConstraint.hpp"

namespace SOArm100::Kinematics::Model
{
class Limits;

class UniversalArticulationBoneConstraint : public ArticulationBoneConstraint
{
public:
UniversalArticulationBoneConstraint( ArticulationConstPtr articulation, BoneConstPtr bone );

virtual void ApplyConstraint(
	const Quaternion& articulation_rotation,
	const Vec3d& articulation_center,
	BoneState& bone_state ) const override;

private:
struct ConstraintSolution
{
	double theta0;
	double theta1;
	AngleAxis rotation0;
	AngleAxis rotation1;
	Vec3d bone_direction;

	double distance_to_solution;
};

ConstraintSolution ComputeSolution(
	double theta0_sol,
	const Limits& joint0_limits,
	const Limits& joint1_limits,
	const Vec3d& a0,
	const Vec3d& a1,
	const Vec3d& b0,
	const Vec3d& b1 ) const;
};
}