#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace SOArm100::Kinematics
{
using Vec2d = Eigen::Vector2d;
using Vec3d = Eigen::Vector3d;
using Vec4d = Eigen::Vector4d;
using Vec6d = Eigen::Matrix< double, 6, 1 >;
using VecXd = Eigen::VectorXd;

using Mat3d = Eigen::Matrix3d;
using Mat4d = Eigen::Matrix4d;
using Mat6d = Eigen::Matrix< double, 6, 6 >;
using MatXd = Eigen::MatrixXd;

using Iso3d = Eigen::Isometry3d;

constexpr double epsilon = 1e-6;

constexpr double error_tolerance = 1e-4;
constexpr double rotation_tolerance = 1e-2;
constexpr double translation_tolerance = 1e-3;
constexpr double gradient_tolerance = 1e-5;
}