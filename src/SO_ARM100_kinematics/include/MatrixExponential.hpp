#pragma once

#include "Twist.hpp"

namespace SOArm100::Kinematics
{
class MatrixExponential
{
public:
  MatrixExponential(const Twist & twist, double theta);
  ~MatrixExponential();

  Eigen::Matrix4d Compute() const;
  operator Eigen::Matrix4d() const;
  Eigen::Matrix4d operator*(const MatrixExponential & other) const;
  Eigen::Matrix4d operator*(const Eigen::Matrix4d matrix) const;

private:
  Twist twist_;
  double theta_;

  Eigen::Matrix3d ComputeRotation(double theta) const;
  Eigen::Vector3d ComputeTranslation(double theta) const;
};
}
