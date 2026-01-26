#include "KinematicsUtils.hpp"

namespace SOArm100::Kinematics
{
// ------------------------------------------------------------

Eigen::Matrix3d SkewMatrix(const Eigen::Vector3d & vec)
{
  Eigen::Matrix3d skew;
  skew << 0, -vec.z(), vec.y(),
    vec.z(), 0, -vec.x(),
    -vec.y(), vec.x(), 0;
  return skew;
}

// ------------------------------------------------------------

void MatrixToPoseMsg(
  const Eigen::Matrix4d & matrix,
  geometry_msgs::msg::Pose & pose_msg)
{
  pose_msg.position.x = matrix(0, 3);
  pose_msg.position.y = matrix(1, 3);
  pose_msg.position.z = matrix(2, 3);

  Eigen::Matrix3d rotation_matrix = matrix.block<3, 3>(0, 0);
  Eigen::Quaterniond quaternion(rotation_matrix);

  pose_msg.orientation.x = quaternion.x();
  pose_msg.orientation.y = quaternion.y();
  pose_msg.orientation.z = quaternion.z();
  pose_msg.orientation.w = quaternion.w();
}

// ------------------------------------------------------------

}
