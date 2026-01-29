#pragma once

#include "Twist.hpp"
#include "Types.hpp"

#include <vector>

namespace SOArm100::Kinematics
{
Mat3d Rotation( const Mat4d& matrix );
Vec3d Translation( const Mat4d& matrix );

Mat3d SkewMatrix( const Vec3d& vec );
Mat6d Adjoint( const Mat4d& transform );

MatXd SpaceJacobian( const std::vector< Twist >& space_twists, const VecXd joint_angles );

MatXd PseudoInverse( const MatXd& jacobian );
MatXd Damped( const MatXd& jacobian, double damping_factor );

Vec6d PoseError( const Mat4d& target, const Mat4d& current );
}
