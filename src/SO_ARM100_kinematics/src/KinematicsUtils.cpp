#include "KinematicsUtils.hpp"

#include "MatrixExponential.hpp"
#include "Types.hpp"

#include <Eigen/Dense>
#include <Eigen/src/Geometry/AngleAxis.h>

namespace SOArm100::Kinematics
{

// ------------------------------------------------------------

void Adjoint( const Mat4d& transform, Mat6d& adjoint ) noexcept
{
	const auto R = Rotation( transform );
	const auto t = transform.block< 3, 1 >( 0, 3 );

	adjoint.setZero();
	adjoint.block< 3, 3 >( 0, 0 ) = R;
	adjoint.block< 3, 3 >( 3, 3 ) = R;
	adjoint.block< 3, 3 >( 3, 0 ) = SkewMatrix( t ) * R;
}

// ------------------------------------------------------------

void SpaceJacobian(
	const std::span< const Twist >& space_twists,
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
		T_cumul *= static_cast< Mat4d >( MatrixExponential( space_twists[i - 1], joint_angles[i - 1] ) );
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
	// Translation
	pose_error.head( 3 ).noalias() = Translation( target ) - Translation( current );

	// Rotation
	Mat3d R_error = Rotation( current ).transpose() * Rotation( target );
	Eigen::AngleAxisd aa_error( R_error );
	pose_error.tail( 3 ).noalias() = aa_error.angle() * aa_error.axis();
}

// ------------------------------------------------------------

}
