#include "Model/Skeleton/UniversalArticulationState.hpp"

#include "Global.hpp"

#include "Model/Joint/Limits.hpp"
#include "Model/Skeleton/BoneState.hpp"
#include "Utils/MathUtils.hpp"
#include "Utils/Distance.hpp"

namespace SOArm100::Kinematics::Model
{

// ------------------------------------------------------------

UniversalArticulationState::UniversalArticulationState( const Articulation* articulation ) :
	ArticulationState( articulation )
{
	assert( articulation->GetType() == ArticulationType::Universal );
}

// ------------------------------------------------------------

void UniversalArticulationState::ApplyConstraints( BoneState& bone_state ) const
{
	auto joint0 = articulation_->Joints()[0];
	auto joint1 = articulation_->Joints()[1];
	auto bone = bone_state.GetBone();
	auto joint_state_0 = joint_states_[0];
	auto joint_state_1 = joint_states_[1];

	const Vec3d& a0 = joint_state_0->Axis();
	const Vec3d& a1 = joint_state_1->Axis();

	const Vec3d& b0 = global_transform_.rotation() * bone->Direction();
	const Vec3d& b1 = bone_state.Direction();

	Vec3d v0;
	Vec3d v1;
	if ( joint0->Axis().cross( bone->Direction() ).norm() < epsilon )
	{
		v0 = a0.cross( a1 );
		v1 = a0.cross( b1 );
	}
	else
	{
		v0 = b0;
		v1 = b1;
	}

	double A = a1.dot( v1 ) - ( a0.dot( v1 ) ) * ( a1.dot( a0 ) );
	double B = -a1.dot( a0.cross( v1 ) );
	double C = ( a0.dot( v1 ) * a1.dot( a0 ) ) - a1.dot( v0 );

	double R = std::sqrt( A * A + B * B );
	double phi = std::atan2( B, A );
	double beta = std::acos( std::clamp( -C / R, -1.0, 1.0 ) );

	auto solution1 = ComputeSolution(
		phi + beta,
		joint0->GetLimits(),
		joint1->GetLimits(),
		a0,
		a1,
		v0,
		v1 );

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
			v0,
			v1 );

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

void UniversalArticulationState::UpdateValues( const BoneState& bone_state  )
{
	auto joint0 = articulation_->Joints()[0];
	auto joint1 = articulation_->Joints()[1];
	auto bone = bone_state.GetBone();
	auto joint_state_0 = joint_states_[0];
	auto joint_state_1 = joint_states_[1];

	Vec3d a0 = joint_state_0->Axis();
	Vec3d a1 = joint_state_1->Axis();

	Vec3d b0 = global_transform_.rotation() * bone->Direction();

	Vec3d v0 = b0;
	Vec3d v1 = bone_state.Direction();

	double A = a1.dot( v1 ) - ( a0.dot( v1 ) ) * ( a1.dot( a0 ) );
	double B = -a1.dot( a0.cross( v1 ) );
	double C = ( a0.dot( v1 ) * a1.dot( a0 ) ) - a1.dot( v0 );

	double R = std::sqrt( A * A + B * B );
	double phi = std::atan2( B, A );
	double beta = std::acos( std::clamp( -C / R, -1.0, 1.0 ) );

	auto solution1 = ComputeSolution(
		phi + beta,
		joint0->GetLimits(),
		joint1->GetLimits(),
		a0,
		a1,
		v0,
		v1 );

	auto solution2 = ComputeSolution(
		phi - beta,
		joint0->GetLimits(),
		joint1->GetLimits(),
		a0,
		a1,
		v0,
		v1 );

	double joint_0_value, joint_1_value;

	if ( std::abs( solution1.distance_to_solution ) > epsilon || std::abs( solution2.distance_to_solution ) > epsilon )
	{
		if ( std::abs( solution1.distance_to_solution ) < std::abs( solution2.distance_to_solution ) )
		{
			joint_0_value = solution1.theta0;
			joint_1_value = solution1.theta1;
		}
		else
		{
			joint_0_value = solution2.theta0;
			joint_1_value = solution2.theta1;
		}
	}

	Vec2d current = { joint_state_0->Value(), joint_state_1->Value() };
	Vec2d sol1 = { solution1.theta0, solution1.theta1 };
	Vec2d sol2 = { solution2.theta0, solution2.theta1 };
	if ( Utils::Distance( sol1, current, Utils::DistanceType::Manhattan ) <
	     Utils::Distance( sol2, current, Utils::DistanceType::Manhattan ) )
	{
		joint_0_value = solution1.theta0;
		joint_1_value = solution1.theta1;
	}
	else
	{
		joint_0_value = solution2.theta0;
		joint_1_value = solution2.theta1;
	}

	local_transform_.setIdentity();
	SetJointInternalState(
		joint_state_0,
		world_transform_,
		global_transform_,
		local_transform_,
		joint_0_value );

	SetJointInternalState(
		joint_state_1,
		world_transform_,
		global_transform_,
		local_transform_,
		joint_1_value );
}

// ------------------------------------------------------------

UniversalArticulationState::Solution UniversalArticulationState::ComputeSolution(
	double theta0_sol,
	const Limits& joint0_limits,
	const Limits& joint1_limits,
	const Vec3d& a0,
	const Vec3d& a1,
	const Vec3d& b0,
	const Vec3d& b1 ) const
{
	Solution solution;

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