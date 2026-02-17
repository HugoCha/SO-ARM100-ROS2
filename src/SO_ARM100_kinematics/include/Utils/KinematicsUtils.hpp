#pragma once

#include "Global.hpp"

#include <span>

namespace SOArm100::Kinematics
{
class JointChain;
struct SolverResult;

[[nodiscard]] inline const Mat3d Rotation( const Mat4d& matrix ) noexcept {
	return matrix.block< 3, 3 >( 0, 0 );
}

[[nodiscard]] inline const Vec3d Translation( const Mat4d& matrix ) noexcept {
	return matrix.block< 3, 1 >( 0, 3 );
}

[[nodiscard]] constexpr const Mat3d SkewMatrix( const Vec3d& vec ) noexcept {
	Mat3d skew;
	skew << 0, -vec.z(), vec.y(),
	    vec.z(), 0, -vec.x(),
	    -vec.y(), vec.x(), 0;
	return skew;
}

[[nodiscard]] const Mat4d Inverse( const Mat4d& transform ) noexcept;

void SpaceJacobian(
	const JointChain& joint_chain,
	const VecXd& joint_angles,
	MatXd& jacobian ) noexcept;

void Adjoint( const Mat4d& transform, Mat6d& adjoint ) noexcept;
void PseudoInverse( const MatXd& jacobian, MatXd& psi ) noexcept;
void Damped( const MatXd& jacobian, double damping_factor, MatXd& damped ) noexcept;

void PoseError( const Mat4d& target, const Mat4d& current, Vec6d& pose_error ) noexcept;
void WeightedPoseError(
	const Mat4d& target,
	const Mat4d& current,
	double rotation_weight,
	double translation_weight,
	Vec6d& pose_error ) noexcept;

void POE(
	const JointChain& joint_chain,
	const Mat4d& M,
	const std::span< const double >& thetas,
	Mat4d& poe );
void POE(
	const JointChain& joint_chain,
	const Mat4d& M,
	const VecXd& thetas,
	Mat4d& poe );

double RotationError( const Mat3d& target_rotation, const Mat3d& result_rotation );
double RotationError( const Mat4d& target, const Mat4d& result );

double PositionError( const Vec3d& target_translation, const Vec3d& result_translation );
double PositionError( const Mat4d& target, const Mat4d& result );

bool IsApprox( 
	const Mat4d& target, 
	const Mat4d& result, 
	double rotation_tol = rotation_tolerance, 
	double translation_tol = translation_tolerance );
	
void CheckSolverResult(
	const JointChain& joint_chain,
	const Mat4d& home_configuration,
	const Mat4d& target,
	Mat4d& result_pose,
	SolverResult& solver_result,
	double rotation_tol = rotation_tolerance,
	double translation_tol = translation_tolerance );
}
