#include "KinematicsSolver.hpp"

#include <cmath>
#include <gtest/gtest.h>
#include <moveit/robot_model/robot_model.hpp>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>

namespace SOArm100::Kinematics::Test
{
class DummyKinematicsSolver : public KinematicsSolver
{
public:
  bool InverseKinematic(
    geometry_msgs::msg::Pose target_pose,
    std::vector<double> & joint_angles) override
  {
    return false;
  }
};

/**
 * Test for a robot model with:
 * - 1 prismatic joint (linear motion)
 * - 1 revolute joint (rotation around z-axis)
 * - 2 revolute joints (rotation around y-axis and z-axis)
 */
class MixedJointKinematicsSolverTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Create URDF string for a simple robot with mixed joint types
    urdf_model_ = urdf::parseURDF(createMixedJointRobotURDF());
    ASSERT_TRUE(urdf_model_ != nullptr) << "Failed to parse URDF";

    // Create SRDF model with arm group
    srdf_model_ = std::make_shared<srdf::Model>();
    srdf_model_->initString(*urdf_model_, createSRDFString());

    // Create RobotModel from URDF and SRDF
    robot_model_ = std::make_shared<moveit::core::RobotModel>(
      urdf_model_, srdf_model_);
    ASSERT_TRUE(robot_model_ != nullptr) << "Failed to create RobotModel";

    // Initialize solver with the robot model
    solver_.Initialize(
      *robot_model_,
      "arm",
      "base_link",
      {"end_effector"},
      0.01);
  }

  void TearDown() override
  {
  }

  std::string createSRDFString() const
  {
    return
      R"(
                <?xml version="1.0"?>
                <robot name="mixed_joint_robot">
                    <group name="arm">
                        <joint name="joint_1_prismatic"/>
                        <joint name="joint_2_revolute_z"/>
                        <joint name="joint_3_revolute_y"/>
                        <joint name="joint_4_revolute_z"/>
                    </group>
                </robot>
            )";
  }

  std::string createMixedJointRobotURDF() const
  {
    return
      R"(
        <?xml version="1.0"?>
        <robot name="mixed_joint_robot">
            <!-- Base link -->
            <link name="base_link">
                <inertial>
                    <mass value="1.0"/>
                    <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
                </inertial>
            </link>

            <!-- Joint 1: Prismatic (vertical motion) -->
            <joint name="joint_1_prismatic" type="prismatic">
                <parent link="base_link"/>
                <child link="link_1"/>
                <origin xyz="0 0 0" rpy="0 0 0"/>
                <axis xyz="0 0 1"/>
                <limit lower="0.0" upper="1.0" effort="100" velocity="1.0"/>
            </joint>

            <link name="link_1">
                <inertial>
                    <mass value="1.0"/>
                    <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
                </inertial>
            </link>

            <!-- Joint 2: Revolute (rotation around z-axis) -->
            <joint name="joint_2_revolute_z" type="revolute">
                <parent link="link_1"/>
                <child link="link_2"/>
                <origin xyz="0 0 0.5" rpy="0 0 0"/>
                <axis xyz="0 0 1"/>
                <limit lower="-3.14159" upper="3.14159" effort="100" velocity="1.0"/>
            </joint>

            <link name="link_2">
                <inertial>
                    <mass value="1.0"/>
                    <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
                </inertial>
            </link>

            <!-- Joint 3: Revolute (rotation around y-axis) -->
            <joint name="joint_3_revolute_y" type="revolute">
                <parent link="link_2"/>
                <child link="link_3"/>
                <origin xyz="0.5 0 0" rpy="0 0 0"/>
                <axis xyz="0 1 0"/>
                <limit lower="-3.14159" upper="3.14159" effort="100" velocity="1.0"/>
            </joint>

            <link name="link_3">
                <inertial>
                    <mass value="1.0"/>
                    <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
                </inertial>
            </link>

            <!-- Joint 4: Revolute (rotation around z-axis) -->
            <joint name="joint_4_revolute_z" type="revolute">
                <parent link="link_3"/>
                <child link="end_effector"/>
                <origin xyz="0.5 0 0" rpy="0 0 0"/>
                <axis xyz="0 0 1"/>
                <limit lower="-3.14159" upper="3.14159" effort="100" velocity="1.0"/>
            </joint>

            <link name="end_effector">
                <inertial>
                    <mass value="0.5"/>
                    <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.005"/>
                </inertial>
            </link>
        </robot>
    )";
  }

protected:
  urdf::ModelInterfaceSharedPtr urdf_model_;
  std::shared_ptr<srdf::Model> srdf_model_;
  moveit::core::RobotModelPtr robot_model_;
  DummyKinematicsSolver solver_;
};

TEST_F(MixedJointKinematicsSolverTest, ForwardKinematicsHomePosition)
{
  std::vector<double> joint_angles = {0.0, 0.0, 0.0, 0.0};
  geometry_msgs::msg::Pose end_effector_pose;

  bool result = solver_.ForwardKinematic(joint_angles, end_effector_pose);

  ASSERT_TRUE(result) << "ForwardKinematic should succeed with home position";

  // At home position, end effector should be at:
  // base_link + link_1 (0m prismatic) + link_2 (0.5m offset) + link_3 (0.5m offset)
  // = (1.0, 0, 0.5) in base frame
  EXPECT_NEAR(end_effector_pose.position.x, 1.0, 0.01);
  EXPECT_NEAR(end_effector_pose.position.y, 0.0, 0.01);
  EXPECT_NEAR(end_effector_pose.position.z, 0.5, 0.01);

  // At home position, orientation should be identity (no rotations)
  EXPECT_NEAR(end_effector_pose.orientation.w, 1.0, 0.01);
  EXPECT_NEAR(end_effector_pose.orientation.x, 0.0, 0.01);
  EXPECT_NEAR(end_effector_pose.orientation.y, 0.0, 0.01);
  EXPECT_NEAR(end_effector_pose.orientation.z, 0.0, 0.01);
}

/**
 * Test ForwardKinematics with prismatic joint extended
 */
TEST_F(MixedJointKinematicsSolverTest, ForwardKinematicsWithPrismaticExtension)
{
  // Extend the prismatic joint by 0.5m
  std::vector<double> joint_angles = {0.5, 0.0, 0.0, 0.0};
  geometry_msgs::msg::Pose end_effector_pose;

  bool result = solver_.ForwardKinematic(joint_angles, end_effector_pose);

  ASSERT_TRUE(result) << "ForwardKinematic should succeed with prismatic extension";

  // With 0.5m prismatic extension, z position should increase
  // base_link + 0.5m (prismatic) + 0.5m (link_2 offset) = 1.0m at z
  EXPECT_NEAR(end_effector_pose.position.x, 1.0, 0.01);
  EXPECT_NEAR(end_effector_pose.position.y, 0.0, 0.01);
  EXPECT_NEAR(end_effector_pose.position.z, 1.0, 0.01);
}

/**
 * Test ForwardKinematics with revolute joint rotation (z-axis)
 */
TEST_F(MixedJointKinematicsSolverTest, ForwardKinematicsWithRevoluteRotationZ)
{
  // Rotate joint_2 (z-axis) by 90 degrees (pi/2 radians)
  std::vector<double> joint_angles = {0.0, 1.5708, 0.0, 0.0};  // pi/2 ≈ 1.5708
  geometry_msgs::msg::Pose end_effector_pose;

  bool result = solver_.ForwardKinematic(joint_angles, end_effector_pose);

  ASSERT_TRUE(result) << "ForwardKinematic should succeed with z-axis rotation";

  // After 90 degree rotation around z-axis from link_2:
  // The 0.5m offset in x should become 0.5m in y (approximately)
  EXPECT_NEAR(end_effector_pose.position.x, 0.0, 0.01);
  EXPECT_NEAR(end_effector_pose.position.y, 1.0, 0.01);
  EXPECT_NEAR(end_effector_pose.position.z, 0.5, 0.01);
}

/**
 * Test ForwardKinematics with revolute joint rotation (y-axis)
 */
TEST_F(MixedJointKinematicsSolverTest, ForwardKinematicsWithRevoluteRotationY)
{
  // Rotate joint_3 (y-axis) by 90 degrees (pi/2 radians)
  std::vector<double> joint_angles = {0.0, 0.0, 1.5708, 0.0};  // pi/2 ≈ 1.5708
  geometry_msgs::msg::Pose end_effector_pose;

  bool result = solver_.ForwardKinematic(joint_angles, end_effector_pose);

  ASSERT_TRUE(result) << "ForwardKinematic should succeed with y-axis rotation";

  // After 90 degree rotation around y-axis at link_3:
  // The 0.5m offset in x should become 0.5m in z
  EXPECT_NEAR(end_effector_pose.position.x, 0.5, 0.01);
  EXPECT_NEAR(end_effector_pose.position.y, 0.0, 0.01);
  EXPECT_NEAR(end_effector_pose.position.z, 1.0, 0.01);
}

/**
 * Test ForwardKinematics with combined joint movements
 */
TEST_F(MixedJointKinematicsSolverTest, ForwardKinematicsWithCombinedMovements)
{
  // Prismatic: 0.25m, joint_2: 45 degrees, joint_3: 0, joint_4: 0
  std::vector<double> joint_angles = {0.25, 0.7854, 0.0, 0.0};  // pi/4 ≈ 0.7854
  geometry_msgs::msg::Pose end_effector_pose;

  bool result = solver_.ForwardKinematic(joint_angles, end_effector_pose);

  ASSERT_TRUE(result) << "ForwardKinematic should succeed with combined movements";

  // Verify that the pose was computed (values should be within reasonable bounds)
  EXPECT_GE(end_effector_pose.position.x, -2.0);
  EXPECT_LE(end_effector_pose.position.x, 2.0);
  EXPECT_GE(end_effector_pose.position.y, -2.0);
  EXPECT_LE(end_effector_pose.position.y, 2.0);
  EXPECT_GE(end_effector_pose.position.z, 0.0);
  EXPECT_LE(end_effector_pose.position.z, 2.0);
}

/**
 * Test ForwardKinematics with all joints in motion
 */
TEST_F(MixedJointKinematicsSolverTest, ForwardKinematicsAllJointsInMotion)
{
  // All joints moving: prismatic, rev_z, rev_y, rev_z
  std::vector<double> joint_angles = {0.5, 1.5708, 0.7854, 0.3927};  // Various angles
  geometry_msgs::msg::Pose end_effector_pose;

  bool result = solver_.ForwardKinematic(joint_angles, end_effector_pose);

  ASSERT_TRUE(result) << "ForwardKinematic should succeed with all joints in motion";

  // Just verify output is reasonable (within robot workspace bounds)
  EXPECT_GE(end_effector_pose.position.x, -3.0);
  EXPECT_LE(end_effector_pose.position.x, 3.0);
  EXPECT_GE(end_effector_pose.position.y, -3.0);
  EXPECT_LE(end_effector_pose.position.y, 3.0);
  EXPECT_GE(end_effector_pose.position.z, 0.0);
  EXPECT_LE(end_effector_pose.position.z, 2.0);

  // Verify orientation is normalized (unit quaternion)
  double quat_magnitude = std::sqrt(
    end_effector_pose.orientation.x * end_effector_pose.orientation.x +
    end_effector_pose.orientation.y * end_effector_pose.orientation.y +
    end_effector_pose.orientation.z * end_effector_pose.orientation.z +
    end_effector_pose.orientation.w * end_effector_pose.orientation.w);
  EXPECT_NEAR(quat_magnitude, 1.0, 0.01) << "Quaternion should be normalized";
}
} // namespace SOArm100::Kinematics::Test
