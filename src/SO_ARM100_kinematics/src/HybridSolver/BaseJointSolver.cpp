#include "HybridSolver/BaseJointSolver.hpp"

#include "Global.hpp"
#include "Joint/JointChain.hpp"
#include "SolverResult.hpp"
#include "Utils/KinematicsUtils.hpp"
#include "Utils/MathUtils.hpp"

#include <cmath>
#include <memory>

namespace SOArm100::Kinematics
{

// ------------------------------------------------------------

BaseJointSolver::BaseJointSolver(
	std::shared_ptr< const JointChain > joint_chain,
	std::shared_ptr< const Mat4d > home_configuration,
	const BaseJointModel& base_joint_model ) :
	joint_chain_( joint_chain )
{
	if ( joint_chain->GetActiveJointCount() == 1 )
		home_configuration_ = home_configuration;
	else
		home_configuration_ = std::make_shared< const Mat4d >( Mat4d::Identity() );

	base_joint_model_ = std::make_unique< const BaseJointModel >( base_joint_model );
}

// ------------------------------------------------------------

void BaseJointSolver::FK(
	const VecXd& joints,
	Mat4d& fk ) const
{
	assert( joint_chain_ );

	const auto& base_twist = GetBaseJoint()->GetTwist();
	fk.noalias() = base_twist.ExponentialMatrix( joints[0] );
}

// ------------------------------------------------------------

SolverResult BaseJointSolver::IK(
	const Mat4d& wrist_center,
	const std::span< const double >& seed_joints,
	double search_discretization ) const
{
	return SolverAnalytical( wrist_center, seed_joints );
}

// ------------------------------------------------------------

SolverResult BaseJointSolver::Heuristic(
	const Mat4d& wrist_center,
	const std::span< const double >& seed_joints,
	double search_discretization ) const
{
	return SolverAnalytical( wrist_center, seed_joints );
}

// ------------------------------------------------------------

SolverResult BaseJointSolver::SolverAnalytical(
	const Mat4d& wrist_center,
	const std::span< const double >& seed_joints ) const
{
	assert( joint_chain_ );

	SolverResult result( 1 );

	const auto* base_joint = GetBaseJoint();

	const Vec3d& omega = base_joint->Axis();
	const Vec3d& t_wrist_center = Translation( wrist_center );
	const Vec3d& r = t_wrist_center - base_joint->Origin();

	// project into plane orthogonal to axis
	const Vec3d& r_proj = ( r - r.dot( omega ) * omega ).normalized();

	if ( r_proj.norm() < epsilon )
	{
		result.joints[0] = seed_joints[0];
		result.state = SolverState::Singularity;
	}
	else
	{
		const auto& r0 = base_joint_model_->reference_direction;
		double s_theta = omega.dot( r0.cross( r_proj ) );
		double c_theta = r0.dot( r_proj );

		double theta = atan2( s_theta, c_theta );
		if ( ValidateAndSelectJoint(
				 base_joint,
				 seed_joints[0],
				 theta,
				 result.joints[0] ) )
		{
			result.state = SolverState::Success;
		}
		else
		{
			result.state = SolverState::Unreachable;
		}
	}

	return result;
}

// ------------------------------------------------------------

const Joint* BaseJointSolver::GetBaseJoint() const
{
	assert( joint_chain_ );
	return joint_chain_->GetActiveJoint( 0 ).get();
}

// ------------------------------------------------------------

bool BaseJointSolver::ValidateAndSelectJoint(
	const Joint* base_joint,
	double seed,
	double theta1,
	double& selection ) const
{
	if ( std::isnan( theta1 ) )
	{
		selection = seed;
		return false;
	}

	double theta2 = 1e10;
	if ( std::fmod( theta1, M_PI ) == 0 )
	{
		if ( base_joint->GetLimits().Within( 0 ) )
			theta2 = 0;
		if ( base_joint->GetLimits().Within( M_PI ) )
			theta2 = FindClosest( seed, theta2, M_PI );
		if ( base_joint->GetLimits().Within( -M_PI ) )
			theta2 = FindClosest( seed, theta2, M_PI );
	}
	else
	{
		theta2 = ( theta1 + M_PI > M_PI ) ? theta1 - M_PI : theta1 + M_PI;
	}

	bool theta1_valid = base_joint->GetLimits().Within( theta1 );
	bool theta2_valid = base_joint->GetLimits().Within( theta2 );

	if ( theta1_valid && theta2_valid )
	{
		selection = FindClosest( seed, theta1, theta2 );
		return true;
	}
	if ( theta1_valid )
	{
		selection = theta1;
		return true;
	}
	if ( theta2_valid )
	{
		selection = theta2;
		return true;
	}

	selection = seed;
	return false;
}

// ------------------------------------------------------------

}