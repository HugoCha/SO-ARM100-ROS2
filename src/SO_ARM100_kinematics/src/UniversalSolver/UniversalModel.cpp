#include "UniversalSolver/UniversalModel.hpp"

#include "Global.hpp"

#include "Model/Joint/Limits.hpp"
#include "UniversalSolver/UniversalSolution.hpp"
#include "Utils/KinematicsUtils.hpp"
#include "Utils/MathUtils.hpp"

#include <limits>

namespace SOArm100::Kinematics::Model
{

// ------------------------------------------------------------

UniversalModel::UniversalModel(
	const Model::JointConstPtr& joint1,
	const Model::JointConstPtr& joint2 )
{
	joints_ = { joint1, joint2 };
}

// ------------------------------------------------------------

std::optional< UniversalModel > UniversalModel::ComputeModel(
	const Model::JointConstPtr& joint1,
	const Model::JointConstPtr& joint2 )
{
	std::vector< Model::JointConstPtr > universal_joints =
	{
		joint1,
		joint2
	};

	if ( !AxesIndependent( universal_joints ) || !ComputeIntersection( universal_joints ).has_value() )
		return std::nullopt;

	return UniversalModel(
		universal_joints[0],
		universal_joints[1] );
}

// ------------------------------------------------------------

Mat3d UniversalModel::Recompose( const Vec2d& q ) const
{
	return ( AngleAxis( q[0], GetJoint( 0 )->Axis() )
	         * AngleAxis( q[1], GetJoint( 1 )->Axis() ) ).toRotationMatrix();
}

// ------------------------------------------------------------

std::vector< Solver::UniversalSolution > UniversalModel::Decompose( const Mat3d& R_target ) const
{
	const Vec3d& a0 = joints_[0]->Axis();
	const Vec3d& a1 = joints_[1]->Axis();

	Vec3d t0 = a1.cross( a0 );
	if ( t0.norm() < 1e-6 )
		t0 = a0.unitOrthogonal();
	t0.normalize();
	Vec3d t1 = a1.cross( t0 ).normalized();

	auto solsA = Decompose( t0, R_target * t0 );
	auto solsB = Decompose( t1, R_target * t1 );

	return { solsA[0], solsA[1], solsB[0], solsB[1] };
}

// ------------------------------------------------------------

std::vector< Solver::UniversalSolution > UniversalModel::Decompose( const Vec3d& b0, const Vec3d& b1 ) const
{
	auto joint0 = joints_[0];
	auto joint1 = joints_[1];

	const auto& l0 = joint0->GetLimits();
	const auto& l1 = joint1->GetLimits();

	const Vec3d& a0 = joint0->Axis();
	const Vec3d& a1 = joint1->Axis();

	if ( b0.cross( a1 ).norm() < 1e-6 )
	{
		double theta0 = AngleAroundAxis( b0, b1, a0 );
		return {
		    ComputeSolution( theta0, l0, l1, a0, a1, b0, b1 )
		};
	}

	double A = a1.dot( b1 ) - ( a0.dot( b1 ) ) * ( a1.dot( a0 ) );
	double B = -a1.dot( a0.cross( b1 ) );
	double C = ( a0.dot( b1 ) * a1.dot( a0 ) ) - a1.dot( b0 );

	double R = std::sqrt( A * A + B * B );
	double phi = std::atan2( B, A );
	double beta = std::acos( std::clamp( -C / ( R + 1e-9 ), -1.0, 1.0 ) );

	return {
	    ComputeSolution( phi - beta, l0, l1, a0, a1, b0, b1 ),
	    ComputeSolution( phi + beta, l0, l1, a0, a1, b0, b1 ),
	};
}

// ------------------------------------------------------------

Solver::UniversalSolution UniversalModel::ComputeSolution(
	double theta0_sol,
	const Limits& l0,
	const Limits& l1,
	const Vec3d& a0,
	const Vec3d& a1,
	const Vec3d& b0,
	const Vec3d& b1 ) const
{
	Solver::UniversalSolution solution;
	solution.reachable = true;
	solution.cost = std::numeric_limits< double >::infinity();

	theta0_sol = WrapAngle( theta0_sol );
	solution.reachable &= l0.Within( theta0_sol );
	solution.angles[0] = l0.Clamp( theta0_sol );
	auto rotation0 = AngleAxis( solution.angles[0], a0 );

	Vec3d a1_prime = rotation0 * a1;
	Vec3d b0_prime = rotation0 * b0;

	double sin_angle = b0_prime.cross( a1_prime ).norm();
	if ( sin_angle < epsilon )
	{
		solution.angles[1] = l1.Center();
		Vec3d local_direction = rotation0 * b0;
		solution.local_rotation = rotation0;
		solution.fk_error = Angle( local_direction.normalized(), b1.normalized() );
	}
	else
	{
		double theta1_sol = AngleAroundAxis( b0_prime, b1, a1_prime );
		theta1_sol = WrapAngle( theta1_sol );
		solution.reachable &= l1.Within( theta1_sol );
		solution.angles[1] = l1.Clamp( theta1_sol );

		auto rotation1 = AngleAxis( solution.angles[1], a1_prime );
		solution.local_rotation = rotation1 * rotation0;
		Vec3d local_direction   = rotation1 * rotation0 * b0;
		solution.fk_error = Angle( local_direction.normalized(), b1.normalized() );
	}

	return solution;
}

// ------------------------------------------------------------

}