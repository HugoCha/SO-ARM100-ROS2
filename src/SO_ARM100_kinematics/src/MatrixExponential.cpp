#include "MatrixExponential.hpp"

#include "KinematicsUtils.hpp"

namespace SOArm100::Kinematics
{
// ------------------------------------------------------------

MatrixExponential::MatrixExponential(const Twist & twist, double theta)
: twist_(twist), theta_(theta)
{}

MatrixExponential::~MatrixExponential()
{}

// ------------------------------------------------------------

Eigen::Matrix4d MatrixExponential::Compute() const
{
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = ComputeRotation(theta_);
  T.block<3, 1>(0, 3) = ComputeTranslation(theta_);
  return T;
}

// ------------------------------------------------------------

MatrixExponential::operator Eigen::Matrix4d() const
{
  return Compute();
}

// ------------------------------------------------------------

Eigen::Matrix4d MatrixExponential::operator*(const MatrixExponential & other) const
{
  return Compute() * other.Compute();
}

// ------------------------------------------------------------

Eigen::Matrix4d MatrixExponential::operator*(const Eigen::Matrix4d matrix) const
{
  return Compute() * matrix;
}

// ------------------------------------------------------------

Eigen::Matrix3d MatrixExponential::ComputeRotation(double theta) const
{
  Eigen::Vector3d w = twist_.GetAxis();
  Eigen::Matrix3d w_hat = SkewMatrix(w);

  Eigen::Matrix3d R = Eigen::Matrix3d::Identity() +
    sin(theta) * w_hat +
    ( 1 - cos(theta) ) * ( w_hat * w_hat );
  return R;
}

// ------------------------------------------------------------

Eigen::Vector3d MatrixExponential::ComputeTranslation(double theta) const
{
  Eigen::Vector3d w = twist_.GetAxis();
  Eigen::Vector3d v = twist_.GetLinear();
  Eigen::Matrix3d w_hat = SkewMatrix(w);

  Eigen::Vector3d t = ( Eigen::Matrix3d::Identity() - ComputeRotation(theta) ) *
    ( w_hat * v ) +
    w * w.transpose() * v * theta;
  return t;
}

// ------------------------------------------------------------
}
