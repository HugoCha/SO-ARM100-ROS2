#pragma once

#include <geometry_msgs/msg/pose.hpp>
#include <Eigen/Dense>

namespace SOArm100::Kinematics
{
Eigen::Matrix3d SkewMatrix(const Eigen::Vector3d & vec);

void MatrixToPoseMsg(
  const Eigen::Matrix4d & matrix,
  geometry_msgs::msg::Pose & pose_msg);
}
