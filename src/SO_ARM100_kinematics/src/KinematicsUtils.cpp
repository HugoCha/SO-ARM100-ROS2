#include "KinematicsUtils.hpp"

#include "MatrixExponential.hpp"
#include "Types.hpp"

#include <Eigen/Dense>
#include <Eigen/src/Geometry/AngleAxis.h>

namespace SOArm100::Kinematics
{
// ------------------------------------------------------------

Mat3d SkewMatrix( const Eigen::Vector3d& vec )
{
	Mat3d skew;
	skew << 0, -vec.z(), vec.y(),
	    vec.z(), 0, -vec.x(),
	    -vec.y(), vec.x(), 0;
	return skew;
}

// ------------------------------------------------------------

Mat3d Rotation( const Mat4d& matrix ) noexcept
{
	return matrix.block< 3, 3 >( 0, 0 );
}

// ------------------------------------------------------------

Vec3d Translation( const Mat4d& matrix ) noexcept
{
	return matrix.block< 3, 1 >( 0, 3 );
}

// ------------------------------------------------------------

void Adjoint( const Mat4d& transform, Mat6d& adjoint ) noexcept
{
	const auto R = Rotation( transform );
	const auto t = transform.block< 3, 1 >( 0, 3 );

	adjoint.setZero();
	adjoint.block< 3, 3 >( 0, 0 ) = R;
	adjoint.block< 3, 3 >( 3, 3 ) = R;
	adjoint.block< 3, 3 >( 0, 3 ) = SkewMatrix( t ) * R;
}

// ------------------------------------------------------------

void SpaceJacobian(
	std::span< const Twist > space_twists,
	const VecXd& joint_angles,
	MatXd& jacobian )
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

Vec6d PoseError( const Mat4d& target, const Mat4d& current ) noexcept
{
	Vec6d error;

	Vec3d t_target = Translation( target );
	Vec3d t_current = Translation( current );
	error.head( 3 ) = t_target - t_current;

	Mat3d R_target = Rotation( target );
	Mat3d R_current = Rotation( current );
	Mat3d R_error = R_current.transpose() * R_target;

	Eigen::AngleAxisd aa_error( R_error );
	error.tail( 3 ) = aa_error.angle() * aa_error.axis();

	return error;
}

// ------------------------------------------------------------

}
