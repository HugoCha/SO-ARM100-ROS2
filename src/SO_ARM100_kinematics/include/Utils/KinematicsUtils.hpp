#pragma once

#include "Global.hpp"

#include <span>

namespace SOArm100::Kinematics
{
class JointChain;
struct JointPose;
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
void WeightedJacobian(
	const MatXd& jacobian,
	const MatXd& weights,
	MatXd& weighted_jacobian ) noexcept;
void JacobianSVD( const MatXd& jacobian, Eigen::JacobiSVD< MatXd >& svd ) noexcept;
void Gradient( const MatXd& jacobian, const VecXd& error, VecXd& gradient );
void Adjoint( const Mat4d& transform, Mat6d& adjoint ) noexcept;
void PseudoInverse( const Eigen::JacobiSVD< MatXd >& svd, double min_sv_tolerance, MatXd& psi ) noexcept;
void PseudoInverse( const MatXd& jacobian, MatXd& psi ) noexcept;
void Damped( const MatXd& jacobian, double damping_factor, MatXd& damped ) noexcept;
void Manipulability( const MatXd& jacobian, MatXd& manipulability ) noexcept;

void PoseError( const Mat4d& target, const Mat4d& current, Vec6d& pose_error ) noexcept;
void WeightedPoseError(
	const Mat4d& target, 
	const Mat4d& current,
	double rotation_weight,
	double translation_weight,
	VecXd& weighted_error ) noexcept;
void ReachableError(
	const Eigen::JacobiSVD< MatXd >& jacobian_svd,
	const VecXd& error,
	double min_sv_tolerance,
	VecXd& reachable_error );

void POE(
	const JointChain& joint_chain,
	const Mat4d& M,
	const std::span< const double >& thetas,
	Mat4d& poe ) noexcept;
void POE(
	const JointChain& joint_chain,
	const Mat4d& M,
	const VecXd& thetas,
	Mat4d& poe ) noexcept;

double RotationError( const Mat3d& target, const Mat3d& result ) noexcept;
double RotationError( const Mat4d& target, const Mat4d& result ) noexcept;

double TranslationError( const Vec3d& target, const Vec3d& result ) noexcept;
double TranslationError( const Mat4d& target, const Mat4d& result ) noexcept;

bool IsApprox(
	const Mat4d& target,
	const Mat4d& result,
	double rotation_tol = rotation_tolerance,
	double translation_tol = translation_tolerance ) noexcept;

void CheckSolverResult(
	const JointChain& joint_chain,
	const Mat4d& home_configuration,
	const Mat4d& target,
	Mat4d& result_pose,
	SolverResult& solver_result,
	double rotation_tol = rotation_tolerance,
	double translation_tol = translation_tolerance ) noexcept;
}
