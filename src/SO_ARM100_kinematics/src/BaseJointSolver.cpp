#include "Global.hpp"

#include "BaseJointSolver.hpp"
#include <cmath>

namespace SOArm100::Kinematics
{

// ------------------------------------------------------------

void BaseJointSolver::FK( const VecXd& base_joint, Mat4d& fk ) const
{
	fk.noalias() = p_base_joint_model_->twist->ExponentialMatrix(base_joint[0]);
}

// ------------------------------------------------------------

BaseJointSolverResult BaseJointSolver::IK(
	const Vec3d& wrist_center,
	const std::span< const double >& seed_joints  ) const
{
	BaseJointSolverResult result;

	const Vec3d& omega = p_base_joint_model_->twist->GetAxis();
	const Vec3d& r = wrist_center - p_base_joint_model_->twist->GetLinear();

	// project into plane orthogonal to axis
	const Vec3d& r_proj = r - r.dot( omega ) * omega;

	if ( r_proj.norm() < epsilon )
	{
		result.base_joint[0] = seed_joints[0];
		result.state = BaseJointSolverState::Singularity;
	}
	else
	{
		const auto& r0 = p_base_joint_model_->reference_direction;
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