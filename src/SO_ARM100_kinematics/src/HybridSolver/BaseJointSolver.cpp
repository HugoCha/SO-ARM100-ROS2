#include "HybridSolver/BaseJointSolver.hpp"

#include "Joint/JointChain.hpp"
#include "Utils/KinematicsUtils.hpp"

#include <cmath>

namespace SOArm100::Kinematics
{

// ------------------------------------------------------------

void BaseJointSolver::Initialize(
	const JointChain& joint_chain,
	const BaseJointModel& base_joint_model )
{
	base_joint = joint_chain.GetActiveJoint( 0 ).get();
	base_joint_model_ = std::make_unique< const BaseJointModel >( base_joint_model );
}

// ------------------------------------------------------------

void BaseJointSolver::FK(
	const VecXd& joints,
	Mat4d& fk ) const
{
	assert( base_joint );

	const auto& base_twist = base_joint->GetTwist();
	fk.noalias() = base_twist.ExponentialMatrix( joints[0] );
}

// ------------------------------------------------------------

BaseJointSolverResult BaseJointSolver::IK(
	const Mat4d& wrist_center,
	const std::span< const double >& seed_joints ) const
{
	assert( base_joint );

	BaseJointSolverResult result;

	const auto& base_twist = base_joint->GetTwist();

	const Vec3d& omega = base_twist.GetAxis();
	const Vec3d& t_wrist_center = Translation( wrist_center );
	const Vec3d& r = t_wrist_center - base_twist.GetLinear();

	// project into plane orthogonal to axis
	const Vec3d& r_proj = r - r.dot( omega ) * omega;

	if ( r_proj.norm() < epsilon )
	{
		result.base_joint[0] = seed_joints[0];
		result.state = BaseJointSolverState::Singularity;
	}
	else
	{
		const auto& r0 = base_joint_model_->reference_direction;
		double s_theta = omega.dot( r0.cross( r_proj ) );
		double c_theta = r0.dot( r_proj );

		result.base_joint[0] = atan2( s_theta, c_theta );
		result.state = std::isnan( result.base_joint[0] ) ?
		               BaseJointSolverState::Unreachable :
		               BaseJointSolverState::Success;
	}

	return result;
}

// ------------------------------------------------------------

}