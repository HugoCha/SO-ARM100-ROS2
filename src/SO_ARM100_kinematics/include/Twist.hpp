#pragma once

#include <Eigen/Dense>

namespace SOArm100::Kinematics
{
class Twist
{
public:
  Twist(Eigen::Vector3d axis, Eigen::Vector3d point_on_axis);
  Twist(Eigen::Vector3d axis, Eigen::Matrix4d transform);
  Twist(const Twist & other);
  ~Twist();

  Eigen::Vector3d GetAxis() const;
  Eigen::Vector3d GetLinear() const;

private:
  Eigen::Vector3d axis_;
  Eigen::Vector3d linear_;
};
}
