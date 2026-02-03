#pragma once

#include "Twist.hpp"
#include "Types.hpp"

#include <span>

namespace SOArm100::Kinematics
{
[[nodiscard]] inline Mat3d Rotation( const Mat4d& matrix ) noexcept {
	return matrix.block< 3, 3 >( 0, 0 );
}

[[nodiscard]] inline Vec3d Translation( const Mat4d& matrix ) noexcept {
	return matrix.block< 3, 1 >( 0, 3 );
}

[[nodiscard]] constexpr Mat3d SkewMatrix( const Vec3d& vec ) noexcept {
	Mat3d skew;
	skew << 0, -vec.z(), vec.y(),
	    vec.z(), 0, -vec.x(),
	    -vec.y(), vec.x(), 0;
	return skew;
}

void SpaceJacobian(
	const std::span< const Twist >& space_twists,
	const VecXd& joint_angles,
	MatXd& jacobian ) noexcept;

void Adjoint( const Mat4d& transform, Mat6d& adjoint ) noexcept;
void PseudoInverse( const MatXd& jacobian, MatXd& psi ) noexcept;
void Damped( const MatXd& jacobian, double damping_factor, MatXd& damped ) noexcept;

void PoseError( const Mat4d& target, const Mat4d& current, Vec6d& pose_error ) noexcept;
}
