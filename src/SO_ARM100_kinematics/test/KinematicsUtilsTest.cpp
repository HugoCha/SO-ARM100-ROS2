#include "KinematicsUtils.hpp"

#include <gtest/gtest.h>

namespace SOArm100::Kinematics::Test
{
class KinematicsUtilsTest : public ::testing::Test {
protected:
  void SetUp() override
  {
  }

  void TearDown() override
  {
  }
};

TEST_F(KinematicsUtilsTest, SkewMatrixTest)
{
    Eigen::Vector3d vec(1.0, 2.0, 3.0);
    Eigen::Matrix3d expected;
    expected << 0, -3.0, 2.0,
                 3.0, 0, -1.0,
                -2.0, 1.0, 0;

    Eigen::Matrix3d result = SOArm100::Kinematics::SkewMatrix(vec);

    ASSERT_TRUE(result.isApprox(expected));
}

TEST_F(KinematicsUtilsTest, MatrixToPoseMsgTest)
{
    Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
    matrix(0, 3) = 1.0;
    matrix(1, 3) = 2.0;
    matrix(2, 3) = 3.0;

    geometry_msgs::msg::Pose pose_msg;
    SOArm100::Kinematics::MatrixToPoseMsg(matrix, pose_msg);

    EXPECT_DOUBLE_EQ(pose_msg.position.x, 1.0);
    EXPECT_DOUBLE_EQ(pose_msg.position.y, 2.0);
    EXPECT_DOUBLE_EQ(pose_msg.position.z, 3.0);
    EXPECT_DOUBLE_EQ(pose_msg.orientation.x, 0.0);
    EXPECT_DOUBLE_EQ(pose_msg.orientation.y, 0.0);
    EXPECT_DOUBLE_EQ(pose_msg.orientation.z, 0.0);
    EXPECT_DOUBLE_EQ(pose_msg.orientation.w, 1.0);
}
}
