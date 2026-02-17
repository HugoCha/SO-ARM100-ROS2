#include "Utils/KinematicsUtils.hpp"

#include "Global.hpp"
#include "Joint/JointChain.hpp"
#include "SolverResult.hpp"

#include <Eigen/Dense>
#include <Eigen/src/Geometry/AngleAxis.h>
#include <cmath>

namespace SOArm100::Kinematics
{

// ------------------------------------------------------------

const Mat4d Inverse( const Mat4d& transform ) noexcept
{
	Mat4d inverse = Mat4d::Identity();
	const Mat3d& R = Rotation( transform );
	const Vec3d& t = Translation( transform );
	inverse.block< 3, 3 >( 0, 0 ).noalias() = R.transpose();
	inverse.block< 3, 1 >( 0, 3 ).noalias() = -R.transpose() * t;
	return inverse;
}

// ------------------------------------------------------------

void Adjoint( const Mat4d& transform, Mat6d& adjoint ) noexcept
{
	const auto R = Rotation( transform );
	const auto t = Translation( transform );

	adjoint.setZero();
	adjoint.block< 3, 3 >( 0, 0 ) = R;
	adjoint.block< 3, 3 >( 3, 3 ) = R;
	adjoint.block< 3, 3 >( 3, 0 ) = SkewMatrix( t ) * R;
}

// ------------------------------------------------------------

void SpaceJacobian(
	const JointChain& joint_chain,
	const VecXd& joint_angles,
	MatXd& jacobian ) noexcept
{
	const size_t n = joint_chain.GetActiveJointCount();
	if ( static_cast< size_t >( jacobian.cols() ) != n )
		jacobian.resize( 6, n );

	Mat4d T_cumul = Mat4d::Identity();
	Mat6d adj_buf;

	const auto& active_joints = joint_chain.GetActiveJoints();

	jacobian.col( 0 ) = static_cast< const Vec6d >( joint_chain.GetActiveJointTwist( 0 ) );
	for ( size_t i = 1; i < n; ++i )
	{
		T_cumul *= joint_chain.GetActiveJointTwist( i - 1 ).ExponentialMatrix( joint_angles[i - 1] );
		Adjoint( T_cumul, adj_buf );
		jacobian.col( i ) = adj_buf * static_cast< const Vec6d >( joint_chain.GetActiveJointTwist( i ) );
	}
}

// ------------------------------------------------------------

void PseudoInverse( const MatXd& jacobian, MatXd& psi ) noexcept
{
	psi.noalias() = jacobian.completeOrthogonalDecomposition().pseudoInverse();
}

// ------------------------------------------------------------

void Damped( const MatXd& jacobian, double damping_factor, MatXd& damped ) noexcept
{
	MatXd Identity = MatXd::Identity( jacobian.cols(), jacobian.cols() );
	damped.noalias() = jacobian.transpose() *
	                   ( jacobian.transpose() * jacobian + damping_factor * Identity );
}

// ------------------------------------------------------------

void PoseError( const Mat4d& target, const Mat4d& current, Vec6d& pose_error ) noexcept
{
	return WeightedPoseError( target, current, 1.0, 1.0, pose_error );
}

// ------------------------------------------------------------

void WeightedPoseError(
	const Mat4d& target,
	const Mat4d& current,
	double rotation_weight,
	double translation_weight,
	Vec6d& pose_error ) noexcept
{
	Mat4d T_diff;
	T_diff.noalias() = target * current.inverse();
	Eigen::AngleAxisd aa( Rotation( T_diff ) );
	pose_error.head( 3 ).noalias() = rotation_weight * aa.axis() * aa.angle();
	pose_error.tail( 3 ).noalias() = translation_weight * Translation( T_diff );
}

// ------------------------------------------------------------

void POE(
	const JointChain& joint_chain,
	const Mat4d& M,
	const std::span< const double >& thetas,
	Mat4d& poe )
{
	assert( joint_chain.GetActiveJointCount() == thetas.size() );
	poe.setIdentity();

	for ( size_t i = 0; i < joint_chain.GetActiveJointCount(); i++ )
	{
		const auto& twist = joint_chain.GetActiveJointTwist( i );
		poe *= twist.ExponentialMatrix( thetas[i] );
	}

	poe *= M;
}

// ------------------------------------------------------------

void POE(
	const JointChain& joint_chain,
	const Mat4d& M,
	const VecXd& thetas,
	Mat4d& poe )
{
	assert( joint_chain.GetActiveJointCount() == thetas.size() );
	poe.setIdentity();

	for ( size_t i = 0; i < joint_chain.GetActiveJointCount(); i++ )
	{
		const auto& twist = joint_chain.GetActiveJointTwist( i );
		poe *= twist.ExponentialMatrix( thetas[i] );
	}

	poe *= M;
}

// ------------------------------------------------------------

double RotationError( const Mat3d& target_rotation, const Mat3d& result_rotation )
{
	Mat3d R_error = target_rotation.transpose() * result_rotation;

	double cos_angle = ( R_error.trace() - 1.0 ) * 0.5;
	cos_angle = std::clamp(cos_angle, -1.0, 1.0);

	return std::acos(cos_angle);
}	

// ------------------------------------------------------------

double RotationError( const Mat4d& target, const Mat4d& result )
{
	return RotationError( Rotation( target ), Rotation( result ) );
}

// ------------------------------------------------------------

double PositionError( const Vec3d& target_translation, const Vec3d& result_translation )
{
	return ( target_translation - result_translation ).norm();
}

// ------------------------------------------------------------

double PositionError( const Mat4d& target, const Mat4d& result )
{
	return PositionError( Translation( target ), Translation( result ) );
}

// ------------------------------------------------------------

bool IsApprox( 
	const Mat4d& target, 
	const Mat4d& result, 
	double rotation_tol, 
	double translation_tol )
{
	return RotationError( target, result ) <= rotation_tol &&
		   PositionError( target, result ) <= translation_tol;
}

// ------------------------------------------------------------

void CheckSolverResult(
	const JointChain& joint_chain,
	const Mat4d& home_configuration,
	const Mat4d& target,
	Mat4d& result_pose,
	SolverResult& solver_result,
	double rot_tolerance,
	double trans_tolerance )
{
	if ( solver_result.Unreachable() )
		return;

	POE( joint_chain, home_configuration, solver_result.joints, result_pose );

	if ( !IsApprox( target, result_pose, rot_tolerance, trans_tolerance ) )
		solver_result.state = SolverState::Unreachable;
}

// ------------------------------------------------------------

}
