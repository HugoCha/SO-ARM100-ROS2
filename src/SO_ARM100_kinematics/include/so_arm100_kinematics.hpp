#pragma once

#include <moveit/kinematics_base/kinematics_base.hpp>

namespace SOArm100::Kinematics
{
class SOArm100AnalyticKinematicsPlugin : public kinematics::KinematicsBase
{
public:
  bool getPositionFK(
    const std::vector<std::string> & link_names,
    const std::vector<double> & joint_angles,
    std::vector<geometry_msgs::msg::Pose> & poses) const override;

  bool getPositionIK(
    const geometry_msgs::msg::Pose & ik_pose,
    const std::vector<double> & ik_seed_state,
    std::vector<double> & solution,
    moveit_msgs::msg::MoveItErrorCodes & error_code,
    const kinematics::KinematicsQueryOptions & options = kinematics::KinematicsQueryOptions()) const
  override;

  bool searchPositionIK(
    const geometry_msgs::msg::Pose & ik_pose,
    const std::vector<double> & ik_seed_state,
    double timeout,
    std::vector<double> & solution, moveit_msgs::msg::MoveItErrorCodes & error_code,
    const kinematics::KinematicsQueryOptions & options = kinematics::KinematicsQueryOptions()) const
  override;

  bool searchPositionIK(
    const geometry_msgs::msg::Pose & ik_pose,
    const std::vector<double> & ik_seed_state,
    double timeout,
    const std::vector<double> & consistency_limits,
    std::vector<double> & solution,
    moveit_msgs::msg::MoveItErrorCodes & error_code,
    const kinematics::KinematicsQueryOptions & options = kinematics::KinematicsQueryOptions()) const
  override;

  bool searchPositionIK(
    const geometry_msgs::msg::Pose & ik_pose,
    const std::vector<double> & ik_seed_state,
    double timeout,
    std::vector<double> & solution,
    const IKCallbackFn & solution_callback,
    moveit_msgs::msg::MoveItErrorCodes & error_code,
    const kinematics::KinematicsQueryOptions & options = kinematics::KinematicsQueryOptions()) const
  override;

  bool searchPositionIK(
    const geometry_msgs::msg::Pose & ik_pose,
    const std::vector<double> & ik_seed_state,
    double timeout,
    const std::vector<double> & consistency_limits,
    std::vector<double> & solution,
    const IKCallbackFn & solution_callback,
    moveit_msgs::msg::MoveItErrorCodes & error_code,
    const kinematics::KinematicsQueryOptions & options = kinematics::KinematicsQueryOptions()) const
  override;

  virtual const std::vector<std::string> & getJointNames() const override;
  virtual const std::vector<std::string> & getLinkNames() const override;

  // initialize
  bool initialize(
    const rclcpp::Node::SharedPtr & node,
    const moveit::core::RobotModel & robot_model,
    const std::string & group_name,
    const std::string & base_frame,
    const std::vector<std::string> & tip_frames,
    double search_discretization) override;

private:
  std::vector<std::string> joint_names_;
  std::vector<std::string> link_names_;
};

}
