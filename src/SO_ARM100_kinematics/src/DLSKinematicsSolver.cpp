#include "DLSKinematicsSolver.hpp"

#include "Converter.hpp"
#include "KinematicsUtils.hpp"
#include "Types.hpp"

#include <algorithm>
#include <Eigen/Dense>
#include <Eigen/src/SVD/JacobiSVD.h>
#include <vector>

namespace SOArm100::Kinematics
{

// ------------------------------------------------------------

DLSKinematicsSolver::DLSKinematicsSolver()
{
}

// ------------------------------------------------------------

DLSKinematicsSolver::~DLSKinematicsSolver()
{
}

// ------------------------------------------------------------

bool DLSKinematicsSolver::InverseKinematic(
	const geometry_msgs::msg::Pose& target_pose,
	std::vector< double >& joint_angles )
{
	Mat4d target = ToMat4d( target_pose );

	double last_error, current_error;
	VecXd last_pose_error, current_pose_error;
	VecXd joints = InitializeJointAngles();
	Mat4d fk;
	int fk_iterations = 0;
	VecXd dq;
	MatXd jac, damped;
	double min_sv_value, damping_factor, step;

	if ( !Initialize( target, joints, fk, jac, last_pose_error, last_error,
	                  damping_factor, step, damped, dq ) )
	{
		joints = RandomValidJointAngles();
		Initialize( target, joints, fk, jac, last_pose_error, last_error,
		            damping_factor, step, damped, dq );
	}

	if ( last_error <= error_tolerance_ )
	{
		joint_angles = ToStdVector( joints );
		return true;
	}

	joints = joints + step * dq;

	for ( int i = 0; i < max_iterations_; i++ )
	{
		if ( !ForwardKinematic( joints, fk ) )
		{
			if ( IsSolverStale( fk_iterations, step, damping_factor ) )
			{
				Initialize( target, RandomValidJointAngles(), fk, jac, last_pose_error,
				            last_error, damping_factor, step, damped, dq );
				fk_iterations = 0;
			}
			else
			{
				step = BacktrackStep( step );
				damping_factor = BacktrackDampingFactor( damping_factor );
				fk_iterations++;
			}
			continue;
		}
		else
		{
			current_pose_error = PoseError( target, fk );
			current_error = current_pose_error.squaredNorm();

			if ( current_error <= error_tolerance_ )
			{
				joint_angles = ToStdVector( joints );
				return true;
			}

			if ( current_error < last_error )
			{
				last_error = current_error;
				last_pose_error = current_pose_error;
				jac = SpaceJacobian( twists_, joints );
				min_sv_value = MinSingularValue( jac );
				damping_factor = AdaptativeDampingFactor( min_sv_value );
				step = AdaptativeStep( min_sv_value );
			}
			else
			{
				step = BacktrackStep( step );
				damping_factor = BacktrackDampingFactor( damping_factor );
			}
		}

		damped = Damped( jac, damping_factor );
		dq = damped.ldlt().solve( jac.transpose() * last_pose_error );
		joints = joints + step * dq;
	}
	return false;
}

// ------------------------------------------------------------

bool DLSKinematicsSolver::Initialize(
	const Mat4d& target,
	const VecXd& initial_joints, Mat4d& fk,
	MatXd& jacobian, VecXd& pose_error,
	double& error, double& damping_factor,
	double& step, MatXd& damped, VecXd& dq )
{
	if ( !ForwardKinematic( initial_joints, fk ) )
	{
		return false;
	}

	jacobian = SpaceJacobian( twists_, initial_joints );
	damping_factor = min_damping_factor_;
	step = 1.0;
	pose_error = PoseError( target, fk );
	error = pose_error.squaredNorm();
	damped = Damped( jacobian, damping_factor );
	dq = damped.ldlt().solve( jacobian.transpose() * pose_error );
	return true;
}

// ------------------------------------------------------------

bool DLSKinematicsSolver::IsSolverStale(
	int fk_iteration, double step,
	double damping_factor )
{
	return ( step <= min_step_ && damping_factor >= max_damping_factor_ ) ||
	       ( fk_iteration >= 5 );
}

// ------------------------------------------------------------

double DLSKinematicsSolver::MinSingularValue( const MatXd& jacobian )
{
	Eigen::JacobiSVD< MatXd > svd( jacobian );
	return std::max( svd.singularValues().minCoeff(), 1e-6 );
}

// ------------------------------------------------------------

double DLSKinematicsSolver::AdaptativeStep( double min_sigma )
{
	double step = min_sigma / ( min_sigma + epsilon_step_ );
	return std::clamp( step, min_step_, 1.0 );
}

// ------------------------------------------------------------

double DLSKinematicsSolver::AdaptativeDampingFactor( double min_sigma )
{
	// min_sigma -> 0 = near singularity
	// exp(-alpha*sigma**2) increase when sigma decrease
	return min_damping_factor_ + ( max_damping_factor_ - min_damping_factor_ ) *
	       exp( -min_sv_factor_ * min_sigma * min_sigma );
}

// ------------------------------------------------------------

double DLSKinematicsSolver::BacktrackStep( double step )
{
	return std::max( step / 2.0, min_step_ );
}

// ------------------------------------------------------------

double DLSKinematicsSolver::BacktrackDampingFactor( double damping )
{
	return std::min( damping * 2.0, max_damping_factor_ );
}

// ------------------------------------------------------------

} // namespace SOArm100::Kinematics
