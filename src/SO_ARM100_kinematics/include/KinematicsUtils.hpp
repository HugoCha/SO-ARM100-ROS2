#pragma once

#include "Twist.hpp"
#include "Types.hpp"

#include <span>

namespace SOArm100::Kinematics
{
[[nodiscard]] Mat3d Rotation( const Mat4d& matrix ) noexcept;
[[nodiscard]] Vec3d Translation( const Mat4d& matrix ) noexcept;

[[nodiscard]] Mat3d SkewMatrix( const Vec3d& vec );

void SpaceJacobian(
	const std::span< const Twist >& space_twists,
	const VecXd& joint_angles,
	MatXd& jacobian );

void Adjoint( const Mat4d& transform, Mat6d& adjoint ) noexcept;
void PseudoInverse( const MatXd& jacobian, MatXd& psi ) noexcept;
void Damped( const MatXd& jacobian, double damping_factor, MatXd& damped ) noexcept;

void PoseError( const Mat4d& target, const Mat4d& current, Vec6d& pose_error ) noexcept;
}
