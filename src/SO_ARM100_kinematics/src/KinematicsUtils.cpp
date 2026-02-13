#include "KinematicsUtils.hpp"

#include <Eigen/Dense>
#include <Eigen/src/Geometry/AngleAxis.h>
#include <cmath>

namespace SOArm100::Kinematics
{

// ------------------------------------------------------------

const Mat4d Inverse( const Mat4d& transform ) noexcept
{
	Mat4d inverse;
	const Mat3d R = Rotation( transform );
	const Vec3d t = Translation( transform );
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
	const std::span< const TwistConstPtr >& space_twists,
	const VecXd& joint_angles,
	MatXd& jacobian ) noexcept
{
	const size_t n = space_twists.size();
	if ( static_cast< size_t >( jacobian.cols() ) != n )
		jacobian.resize( 6, n );

	Mat4d T_cumul = Mat4d::Identity();
	Mat6d adj_buf;

	jacobian.col( 0 ) = static_cast< Vec6d >( space_twists[0] );
	for ( size_t i = 1; i < n; ++i )
	{
		T_cumul *= space_twists[i - 1]->ExponentialMatrix( joint_angles[i - 1] );
		Adjoint( T_cumul, adj_buf );
		jacobian.col( i ) = adj_buf * static_cast< Vec6d >( space_twists[i] );
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
	const std::span< const TwistConstPtr >& twists, 
	const std::span< const double >& thetas, 
	const Mat4d& M,
	Mat4d& poe )
{
	assert( twists.size() == thetas.size() );
	poe.setIdentity();

	for ( auto i = 0; i < twists.size(); i++ )
	{
		poe.noalias() = poe * twists[i]->ExponentialMatrix(thetas[i]);
	}

	poe.noalias() = poe * M;
}

// ------------------------------------------------------------

void POE( 	
	const std::span< const TwistConstPtr >& twists, 
	const VecXd& thetas, 
	const Mat4d& M,
	Mat4d& poe )
{
	assert( twists.size() == thetas.size() );
	poe.setIdentity();

	for ( auto i = 0; i < twists.size(); i++ )
	{
		poe.noalias() = poe * twists[i]->ExponentialMatrix(thetas[i]);
	}

	poe.noalias() = poe * M;
}

// ------------------------------------------------------------

}
