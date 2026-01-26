#pragma once

#include "Twist.hpp"

#include <Eigen/src/Core/Matrix.h>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/macros/class_forward.hpp>

#include <vector>

namespace moveit::core
{
MOVEIT_CLASS_FORWARD(RobotModel);
}

namespace SOArm100::Kinematics
{
class KinematicsSolver
{
public:
  KinematicsSolver();
  ~KinematicsSolver();

  void Initialize(
    const moveit::core::RobotModel & robot_model,
    const std::string & group_name,
    const std::string & base_frame,
    const std::vector<std::string> & tip_frames,
    double search_discretization);

  bool ForwardKinematic(std::vector<double> joint_angles, geometry_msgs::msg::Pose & pose);
  virtual bool InverseKinematic(
    geometry_msgs::msg::Pose target_pose,
    std::vector<double> & joint_angles) = 0;

protected:
  moveit::core::RobotModelConstPtr robot_model_;
  std::vector<Twist> twists_;
  Eigen::Matrix4d home_configuration_;
};
}
