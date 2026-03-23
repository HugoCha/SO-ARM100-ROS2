#include "Utils/KinematicsUtils.hpp"

#include "Global.hpp"
#include "Model/JointChain.hpp"

#include <Eigen/Dense>
#include <Eigen/src/Geometry/AngleAxis.h>
#include <cmath>
#include <stdexcept>

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
	const Model::JointChain& joint_chain,
	const VecXd& joint_angles,
	MatXd& jacobian ) noexcept
{
	const size_t n = joint_chain.GetActiveJointCount();
	if ( static_cast< size_t >( jacobian.cols() ) != n )
		jacobian.resize( 6, n );

	Mat4d T_cumul = Mat4d::Identity();
	Mat6d adj_buf;

	const auto& active_joints = joint_chain.GetActiveJoints();

	jacobian.col( 0 ).noalias() = static_cast< const Vec6d >( joint_chain.GetActiveJointTwist( 0 ) );
	for ( size_t i = 1; i < n; ++i )
	{
		T_cumul *= joint_chain.GetActiveJointTwist( i - 1 ).ExponentialMatrix( joint_angles[i - 1] );
		Adjoint( T_cumul, adj_buf );
		jacobian.col( i ).noalias() = adj_buf * static_cast< const Vec6d >( joint_chain.GetActiveJointTwist( i ) );
	}
}

// ------------------------------------------------------------

void WeightedJacobian(
	const MatXd& jacobian,
	const MatXd& weights,
	MatXd& weighted_jacobian ) noexcept
{
	const int dofs = weights.rows();
	const int n_joints = jacobian.cols();
	if ( weighted_jacobian.rows() != dofs || weighted_jacobian.cols() != n_joints )
		weighted_jacobian.resize( dofs, n_joints );

	weighted_jacobian.noalias() = weights * jacobian;
}

// ------------------------------------------------------------

void JacobianSVD( const MatXd& jacobian, Eigen::JacobiSVD< MatXd >& svd ) noexcept
{
	svd = Eigen::JacobiSVD< MatXd >(
		jacobian,
		Eigen::ComputeThinU | Eigen::ComputeThinV );
}

// ------------------------------------------------------------

void Gradient( const MatXd& jacobian, const VecXd& error, VecXd& gradient )
{
	const int task_dofs = jacobian.rows();
	const int n_joints  = jacobian.cols();

	if ( task_dofs != error.rows() )
		throw std::runtime_error( "Size mismatch between jacobian and error" );
	if ( n_joints != gradient.rows() )
		gradient.resize( n_joints );

	gradient.noalias() = jacobian.transpose() * error;
}

// ------------------------------------------------------------

void PseudoInverse( const Eigen::JacobiSVD< MatXd >& svd,
                    double min_sv_tolerance,
                    MatXd& psi ) noexcept
{
	auto singularValues = svd.singularValues();
	Eigen::VectorXd invertedSingularValues = Eigen::VectorXd::Zero( singularValues.size() );

	for ( int i = 0; i < singularValues.size(); ++i )
		if ( singularValues( i ) > min_sv_tolerance )
			invertedSingularValues( i ) = 1.0 / singularValues( i );

	psi.noalias() = svd.matrixV() * invertedSingularValues.asDiagonal() * svd.matrixU().transpose();
}

// ------------------------------------------------------------

void PseudoInverse( const MatXd& jacobian, MatXd& psi ) noexcept
{
	psi.noalias() = jacobian.completeOrthogonalDecomposition().pseudoInverse();
}

// ------------------------------------------------------------

void Damped( const MatXd& jacobian, double damping_factor, MatXd& damped ) noexcept
{
	const int n_joints = jacobian.cols();
	if ( damped.cols() != n_joints || damped.rows() != n_joints )
		damped.resize( n_joints, n_joints );

	damped.noalias() = jacobian.transpose() * jacobian;
	damped.diagonal().array() += damping_factor * damping_factor;
}

// ------------------------------------------------------------

void Manipulability( const MatXd& jacobian, MatXd& manipulability ) noexcept
{
	const int dofs = jacobian.rows();
	if ( manipulability.cols() != dofs || manipulability.rows() != dofs )
		manipulability.resize( dofs, dofs );

	manipulability.noalias() = jacobian * jacobian.transpose();
}

// ------------------------------------------------------------

void PoseError(
	const Mat4d& target,
	const Mat4d& current,
	Vec6d& pose_error ) noexcept
{
	Mat4d T_error = target * Inverse( current );
	Eigen::AngleAxisd aa( Rotation( T_error ) );
	Vec3d omega = aa.axis() * aa.angle();
	pose_error.head( 3 ).noalias() = omega;
	pose_error.tail( 3 ).noalias() = Translation( T_error );
}

// ------------------------------------------------------------

void WeightedPoseError(
	const Mat4d& target,
	const Mat4d& current,
	double rotation_weight,
	double translation_weight,
	VecXd& weighted_error ) noexcept
{
	assert( rotation_weight > 0 || translation_weight > 0 );
	assert( weighted_error.size() >= 3 && ( rotation_weight <= 0 || translation_weight <= 0 ) ||
	        weighted_error.size() == 6 && rotation_weight > 0 && translation_weight > 0 );

	weighted_error.setZero();

	Mat4d T_error = target * Inverse( current );

	if ( rotation_weight > 0 )
	{
		Eigen::AngleAxisd aa( Rotation( T_error ) );
		weighted_error.head( 3 ).noalias() =
			rotation_weight * ( aa.axis() * aa.angle() );
	}
	if ( translation_weight > 0 )
	{
		weighted_error.tail( 3 ).noalias() = translation_weight * Translation( T_error );
	}
}

// ------------------------------------------------------------

void ReachableError(
	const Eigen::JacobiSVD< MatXd >& jacobian_svd,
	const VecXd& error,
	double min_sv_tolerance,
	VecXd& reachable_error )
{
	const int dofs = jacobian_svd.rows();
	if ( dofs != error.rows() )
		throw std::runtime_error( "Size mismatch between jacobian svd and error" );
	if ( dofs != reachable_error.rows() )
		reachable_error.resize( dofs );

	VecXd sigma = jacobian_svd.singularValues();
	MatXd U = jacobian_svd.matrixU();
	int rank = ( sigma.array() > min_sv_tolerance ).count();
	MatXd U_proj = U.leftCols( rank );
	VecXd e_proj = U_proj.transpose() * error;
	if ( rank == 0 )
	{
		reachable_error.noalias() = VecXd::Zero( error.size() );
	}
	else
	{
		reachable_error.noalias() = U_proj * e_proj;
	}
}

// ------------------------------------------------------------

void POE(
	const Model::JointChain& joint_chain,
	const Mat4d& M,
	const std::span< const double >& thetas,
	Mat4d& poe ) noexcept
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
	const Model::JointChain& joint_chain,
	const Mat4d& M,
	const VecXd& thetas,
	Mat4d& poe ) noexcept
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

double RotationError( const Mat3d& target, const Mat3d& result ) noexcept
{
	Mat3d R_error = result.transpose() * target;
	return acos( ( R_error.trace() - 1 ) / 2.0 );
}

// ------------------------------------------------------------

double RotationError( const Mat4d& target, const Mat4d& result ) noexcept
{
	return RotationError( Rotation( target ), Rotation( result ) );
}

// ------------------------------------------------------------

double TranslationError( const Vec3d& target, const Vec3d& result ) noexcept
{
	return ( target - result ).norm();
}

// ------------------------------------------------------------

double TranslationError( const Mat4d& target, const Mat4d& result ) noexcept
{
	return TranslationError( Translation( target ), Translation( result ) );
}

// ------------------------------------------------------------

bool IsApprox(
	const Mat4d& target,
	const Mat4d& result,
	double rotation_tol,
	double translation_tol ) noexcept
{
	return RotationError( target, result ) <= rotation_tol &&
	       TranslationError( target, result ) <= translation_tol;
}

// ------------------------------------------------------------

std::vector< double > EvaluateAngleCandidates(
	const Model::Joint& joint,
	double seed,
	double raw_angle ) noexcept
{
	if ( std::isnan( raw_angle ) )
		return {}
	;

	std::vector< double > candidates;
	candidates.reserve( 2 );

	const auto& limits = joint.GetLimits();

	double mirror = raw_angle + M_PI;
	if ( mirror >  M_PI )
		mirror -= 2.0 * M_PI;
	if ( mirror < -M_PI )
		mirror += 2.0 * M_PI;

	if ( limits.Within( raw_angle ) )
		candidates.push_back( raw_angle );

	if ( limits.Within( mirror ) )
		candidates.push_back( mirror );

	auto wrap_dist = []( double a, double b ) noexcept {
						 double d = std::abs( a - b );
						 return std::min( d, 2.0 * M_PI - d );
					 };

	std::sort( candidates.begin(), candidates.end(),
	           [&]( double a, double b ){
			return wrap_dist( a, seed ) < wrap_dist( b, seed );
		} );

	return candidates;
}

// ------------------------------------------------------------

}
