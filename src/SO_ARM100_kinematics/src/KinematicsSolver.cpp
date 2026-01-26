#include "KinematicsSolver.hpp"
#include "KinematicsUtils.hpp"
#include "MatrixExponential.hpp"

#include <moveit/robot_model/joint_model.hpp>
#include <moveit/robot_model/link_model.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <rclcpp/logger.hpp>

namespace SOArm100::Kinematics
{

static rclcpp::Logger logger = rclcpp::get_logger("KinematicsSolver");

// ------------------------------------------------------------

KinematicsSolver::KinematicsSolver()
: robot_model_(nullptr) {}

// ------------------------------------------------------------

KinematicsSolver::~KinematicsSolver() {}

// ------------------------------------------------------------

void KinematicsSolver::Initialize(
  const moveit::core::RobotModel & robot_model,
  const std::string & group_name,
  const std::string & base_frame,
  const std::vector<std::string> & tip_frames,
  double search_discretization)
{
  RCLCPP_INFO(logger, "Initializing KinematicsSolver for group: %s",
              group_name.c_str());
  robot_model_ = std::make_shared<moveit::core::RobotModel>(robot_model);

  moveit::core::RobotState state(robot_model_);
  state.setToDefaultValues();

  auto joint_group = robot_model_->getJointModelGroup(group_name);
  if (!joint_group) {
    RCLCPP_ERROR(logger, "Joint model group %s not found in robot model.",
                 group_name.c_str());
    return;
  }

  const auto & joint_models = joint_group->getActiveJointModels();

  home_configuration_ = state.getGlobalLinkTransform(tip_frames[0]).matrix();
  twists_.clear();
  for (const auto & joint_model : joint_models) {
    if (joint_model->getType() != moveit::core::JointModel::REVOLUTE &&
      joint_model->getType() != moveit::core::JointModel::PRISMATIC &&
      joint_model->getType() != moveit::core::JointModel::FIXED)
    {
      RCLCPP_WARN(logger,
                  "Joint %s is not fixed, revolute or prismatic. Skipping.",
                  joint_model->getName().c_str());
      continue;
    }

    Eigen::Vector3d axis;
    if (joint_model->getType() == moveit::core::JointModel::FIXED) {
      axis = Eigen::Vector3d::Zero();
    } else if (joint_model->getType() == moveit::core::JointModel::REVOLUTE) {
      const auto *revolute_joint_model =
        static_cast<const moveit::core::RevoluteJointModel *>(joint_model);
      axis = revolute_joint_model->getAxis();
    } else if (joint_model->getType() == moveit::core::JointModel::PRISMATIC) {
      const auto *prismatic_joint_model =
        static_cast<const moveit::core::PrismaticJointModel *>(joint_model);
      axis = prismatic_joint_model->getAxis();
    }

    auto joint_transform =
      state.getGlobalLinkTransform(joint_model->getParentLinkModel());
    Eigen::Matrix3d R = joint_transform.rotation();
    Eigen::Vector3d point_on_axis_world = joint_transform.translation();
    Eigen::Vector3d axis_world = R * axis;

    auto twist = Twist(axis_world, point_on_axis_world);
    twists_.push_back(twist);
  }
}

// ------------------------------------------------------------

bool KinematicsSolver::ForwardKinematic(
  std::vector<double> joint_angles,
  geometry_msgs::msg::Pose & pose)
{
  if (!robot_model_) {
    RCLCPP_ERROR(logger, "Robot model not initialized.");
    return false;
  }

  if (joint_angles.size() != twists_.size()) {
    RCLCPP_ERROR(
        logger,
        "Joint angles size (%zu) does not match number of twists (%zu).",
        joint_angles.size(), twists_.size());
    return false;
  }

  Eigen::Matrix4d T_end = Eigen::Matrix4d::Identity();
  for (size_t i = 0; i < twists_.size(); ++i) {
    Eigen::Matrix4d exp_twist = MatrixExponential(twists_[i], joint_angles[i]);
    T_end = T_end * exp_twist;
  }
  T_end = T_end * home_configuration_;

  MatrixToPoseMsg(T_end, pose);
  return true;
}

} // namespace SOArm100::Kinematics
