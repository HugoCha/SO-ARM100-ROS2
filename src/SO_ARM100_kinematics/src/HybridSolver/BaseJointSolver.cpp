#include "HybridSolver/BaseJointSolver.hpp"

#include "Global.hpp"
#include "Joint/JointChain.hpp"
#include "SolverResult.hpp"
#include "Utils/MathUtils.hpp"
#include "Utils/KinematicsUtils.hpp"

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
	assert( joint_chain_ );

	SolverResult result( 1 );

	const auto& base_joint = GetBaseJoint();
	const Vec3d& base_origin = GetBaseJoint()->Origin();
	const Vec3d& base_axis = GetBaseJoint()->Axis();
	
	const Vec3d& t_wrist_center = Translation( wrist_center );
	const Vec3d& r = t_wrist_center - base_origin;

	// project into plane orthogonal to axis
	const Vec3d& r_proj = r - r.dot( base_axis ) * base_axis;

	if ( r_proj.norm() < epsilon )
	{
		result.joints[0] = seed_joints[0];
		result.state = SolverState::Singularity;
	}
	else
	{
		const auto& r0 = base_joint_model_->reference_direction;
		double s_theta = base_axis.dot( r0.cross( r_proj ) );
		double c_theta = r0.dot( r_proj );

		double joint_value = atan2( s_theta, c_theta );
		if ( std::isnan( joint_value ) )
		{
			result.state = SolverState::Unreachable;
		}
		else
		{
			result.state = SolverState::Success;
			result.joints[0] = findClosest( seed_joints[0], joint_value, M_PI - joint_value );
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

}