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

	const Vec3d& a0 = joint0->Axis();
	const Vec3d& a1 = joint1->Axis();

	const Vec3d& b0 = bone->Direction();
	const Vec3d& b1 = world_transform_.rotation().inverse() * bone_state.Direction();

	double A = a1.dot( b1 ) - ( a0.dot( b1 ) ) * ( a1.dot( a0 ) );
	double B = -a1.dot( a0.cross( b1 ) );
	double C = ( a0.dot( b1 ) * a1.dot( a0 ) ) - a1.dot( b0 );

	double R = std::sqrt( A * A + B * B );
	double phi = std::atan2( B, A );
	double beta = std::acos( std::clamp( -C / ( R + 1e-9 ), -1.0, 1.0 ) );

	auto solution1 = ComputeSolution(
		phi - beta,
		bone,
		joint0->GetLimits(),
		joint1->GetLimits(),
		a0,
		a1,
		b0,
		b1 );

	if ( std::abs( solution1.distance_to_solution ) < epsilon )
	{
		bone_state.Direction() = world_transform_.rotation() * solution1.local_bone_direction;
	}
	else
	{
		auto solution2 = ComputeSolution(
			phi + beta,
			bone,
			joint0->GetLimits(),
			joint1->GetLimits(),
			a0,
			a1,
			b0,
			b1 );

		if ( std::abs( solution2.distance_to_solution ) < std::abs( solution1.distance_to_solution ) )
		{
			bone_state.Direction() = world_transform_.rotation() * solution2.local_bone_direction;
		}
		else
		{
			bone_state.Direction() = world_transform_.rotation() * solution1.local_bone_direction;
		}
	}
}

// ------------------------------------------------------------

void UniversalArticulationState::UpdateValues( 
	const BoneState& bone_state, 
	double damping_factor )
{
	auto joint0 = articulation_->Joints()[0];
	auto joint1 = articulation_->Joints()[1];
	auto bone = bone_state.GetBone();
	auto joint_state_0 = joint_states_[0];
	auto joint_state_1 = joint_states_[1];

	Vec3d a0 = joint0->Axis();
	Vec3d a1 = joint1->Axis();

	const Vec3d& b0 = bone->Direction();
	const Vec3d& b1 = world_transform_.rotation().inverse() * bone_state.Direction();

	double A = a1.dot( b1 ) - ( a0.dot( b1 ) ) * ( a1.dot( a0 ) );
	double B = -a1.dot( a0.cross( b1 ) );
	double C = ( a0.dot( b1 ) * a1.dot( a0 ) ) - a1.dot( b0 );

	double R = std::sqrt( A * A + B * B );
	double phi = std::atan2( B, A );
	double beta = std::acos( std::clamp( -C / ( R + 1e-9 ), -1.0, 1.0 ) );

	auto solution1 = ComputeSolution(
		phi - beta,
		bone,
		joint0->GetLimits(),
		joint1->GetLimits(),
		a0,
		a1,
		b0,
		b1 );

	auto solution2 = ComputeSolution(
		phi + beta,
		bone,
		joint0->GetLimits(),
		joint1->GetLimits(),
		a0,
		a1,
		b0,
		b1 );

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
	else
	{
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
	}

	Iso3d local_transform = Iso3d::Identity();

	SetJointInternalState(
		joint_state_0,
		local_transform,
		joint_0_value,
		damping_factor );

	SetJointInternalState(
		joint_state_1,
		local_transform,
		joint_1_value,
		damping_factor );

	local_transform_ = local_transform;
	global_transform_ = world_transform_ * local_transform_;
}

// ------------------------------------------------------------

UniversalArticulationState::Solution UniversalArticulationState::ComputeSolution(
	double theta0_sol,
	Model::BoneConstPtr bone,
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
	
	Vec3d local_direction = solution.rotation1 * solution.rotation0 * b0;
	solution.distance_to_solution = Angle( local_direction, b1 );
	
	solution.local_bone_direction = 
		( solution.rotation1 * solution.rotation0 * bone->Direction() ).normalized() * bone->Length();
	
	return solution;
}

// ------------------------------------------------------------

}