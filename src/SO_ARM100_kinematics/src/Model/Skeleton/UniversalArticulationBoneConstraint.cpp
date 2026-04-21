#include "Model/Skeleton/UniversalArticulationBoneConstraint.hpp"

#include "Global.hpp"

#include "Model/Joint/Limits.hpp"
#include "Model/Skeleton/BoneState.hpp"
#include "Utils/MathUtils.hpp"

namespace SOArm100::Kinematics::Model
{

// ------------------------------------------------------------

UniversalArticulationBoneConstraint::UniversalArticulationBoneConstraint(
	ArticulationConstPtr articulation,
	BoneConstPtr bone ) :
	ArticulationBoneConstraint( articulation, bone )
{
	assert( articulation->GetType() == ArticulationType::Universal );
}

// ------------------------------------------------------------

void UniversalArticulationBoneConstraint::ApplyConstraint(
	const Quaternion& articulation_rotation,
	const Vec3d& articulation_center,
	BoneState& bone_state ) const
{
	auto joint0 = articulation_->Joints()[0];
	auto joint1 = articulation_->Joints()[1];

	Vec3d a0 = articulation_rotation * joint0->Axis();
	Vec3d a1 = articulation_rotation * joint1->Axis();

	Vec3d b0 = articulation_rotation * bone_->Direction();
	Vec3d b1 = bone_state.Direction();

	double A = a1.dot( b1 ) - ( a0.dot( b1 ) ) * ( a1.dot( a0 ) );
	double B = -a1.dot( a0.cross( b1 ) );
	double C = ( a0.dot( b1 ) * a1.dot( a0 ) ) - a1.dot( b0 );

	double R = std::sqrt( A * A + B * B );
	double phi = std::atan2( B, A );
	double beta = std::acos( std::clamp( -C / R, -1.0, 1.0 ) );

	auto solution1 = ComputeSolution(
		phi + beta,
		joint0->GetLimits(),
		joint1->GetLimits(),
		a0,
		a1,
		b0,
		b1 );

	if ( std::abs( solution1.distance_to_solution ) < epsilon )
	{
		bone_state.Direction() = solution1.bone_direction;
	}
	else
	{
		auto solution2 = ComputeSolution(
			phi - beta,
			joint0->GetLimits(),
			joint1->GetLimits(),
			a0,
			a1,
			b0,
			b1 );

		if ( std::abs( solution2.distance_to_solution ) < std::abs( solution1.distance_to_solution ) )
		{
			bone_state.Direction() = solution2.bone_direction;
		}
		else
		{
			bone_state.Direction() = solution1.bone_direction;
		}
	}
}

// ------------------------------------------------------------

UniversalArticulationBoneConstraint::ConstraintSolution UniversalArticulationBoneConstraint::ComputeSolution(
	double theta0_sol,
	const Limits& joint0_limits,
	const Limits& joint1_limits,
	const Vec3d& a0,
	const Vec3d& a1,
	const Vec3d& b0,
	const Vec3d& b1 ) const
{
	ConstraintSolution solution;

	theta0_sol = std::remainder( theta0_sol, 2 * M_PI );

	solution.theta0 = joint0_limits.Clamp( theta0_sol );
	solution.rotation0 = AngleAxis( solution.theta0, a0 );

	auto rotation0_sol = AngleAxis( theta0_sol, a0 );
	Vec3d a1_prime = rotation0_sol * a1;
	Vec3d b0_prime = rotation0_sol * b0;
	double theta1_sol = AngleAroundAxis( b0_prime, b1, a1_prime );

	solution.theta1 = joint1_limits.Clamp( theta1_sol );
	solution.rotation1 = AngleAxis( solution.theta1, a1_prime );
	solution.bone_direction = solution.rotation1 * solution.rotation0 * b0;

	if ( solution.theta0 == theta0_sol && solution.theta1 == theta1_sol )
		solution.distance_to_solution = 0;
	else
		solution.distance_to_solution = Angle( b1, solution.bone_direction );

	return solution;
}

// ------------------------------------------------------------

}