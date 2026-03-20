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

constexpr double large_rotation_error  = 18 * M_PI / 180; // rad
constexpr double medium_rotation_error  =  9 * M_PI / 180; // rad
constexpr double small_rotation_error  =  3 * M_PI / 180; // rad

constexpr double large_translation_error  = 0.12;  // m
constexpr double medium_translation_error  = 0.03; // m
constexpr double small_translation_error  = 0.01; // m

constexpr double error_tolerance = 1e-3;
constexpr double rotation_tolerance = 1e-2;
constexpr double translation_tolerance = 1e-3;
constexpr double gradient_tolerance = 1e-6;

constexpr double timeout = 100; // ms
}