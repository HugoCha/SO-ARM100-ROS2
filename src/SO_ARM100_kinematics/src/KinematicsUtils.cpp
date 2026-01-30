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

Mat3d Rotation( const Mat4d& matrix )
{
	return matrix.block< 3, 3 >( 0, 0 );
}

// ------------------------------------------------------------

Vec3d Translation( const Mat4d& matrix )
{
	return matrix.block< 3, 1 >( 0, 3 );
}

// ------------------------------------------------------------

Mat6d Adjoint( const Mat4d& transform )
{
	Mat6d Adj = Mat6d::Zero();

	Mat3d R = Rotation( transform );
	Vec3d t = Translation( transform );

	Mat3d skew = SkewMatrix( t );

	Adj.block< 3, 3 >( 0, 0 ) = R;
	Adj.block< 3, 3 >( 0, 3 ) = skew * R;
	Adj.block< 3, 3 >( 3, 0 ) = Mat3d::Zero();
	Adj.block< 3, 3 >( 3, 3 ) = R;

	return Adj;
}

// ------------------------------------------------------------

MatXd SpaceJacobian( const std::vector< Twist >& space_twists, const VecXd& joint_angles )
{
	int n = space_twists.size();
	if ( n != joint_angles.size() )
	{
		throw std::invalid_argument(
				  "Size mismatch: expected " + std::to_string( n ) +
				  " elements, got " + std::to_string( joint_angles.size()));
	}

	MatXd space_jac( 6, n );

	Mat4d T_cumul = Mat4d::Identity();
	Mat4d T_inter;
	Mat6d adjoint;

	space_jac.col( 0 ) = ( Vec6d )space_twists[0];
	for ( int i = 1; i < n; i++ )
	{
		T_inter = MatrixExponential( space_twists[i - 1], joint_angles[i - 1] );
		T_cumul *= T_inter;
		adjoint = Adjoint( T_cumul );
		space_jac.col( i ) = adjoint * ( Vec6d )space_twists[i];
	}

	return space_jac;
}

// ------------------------------------------------------------

MatXd PseudoInverse( const MatXd& jacobian )
{
	return jacobian.completeOrthogonalDecomposition().pseudoInverse();
}

// ------------------------------------------------------------

MatXd Damped( const MatXd& jacobian, double damping_factor )
{
	MatXd Identity = MatXd::Identity( jacobian.cols(), jacobian.cols());
	return jacobian.transpose() *
	       ( jacobian.transpose() * jacobian + damping_factor * Identity );
}

// ------------------------------------------------------------

Vec6d PoseError( const Mat4d& target, const Mat4d& current )
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
