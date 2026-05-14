#pragma once

#include "Global.hpp"

#include "ArticulationState.hpp"

namespace SOArm100::Kinematics::Model
{
class Limits;

class UniversalArticulationState : public ArticulationState
{
public:
UniversalArticulationState( const Articulation* articulation );

virtual void ApplyConstraints( BoneState& bone_state ) const override;
virtual void UpdateValues( const BoneState& bone_state ) override;

private:
struct Solution
{
	double theta0;
	double theta1;
	AngleAxis rotation0;
	AngleAxis rotation1;
	Vec3d bone_direction;

	double distance_to_solution;
};

Solution ComputeSolution(
	double theta0_sol,
	const Limits& joint0_limits,
	const Limits& joint1_limits,
	const Vec3d& a0,
	const Vec3d& a1,
	const Vec3d& b0,
	const Vec3d& b1 ) const;
};
}