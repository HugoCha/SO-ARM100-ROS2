#include "Twist.hpp"

#include <gtest/gtest.h>

namespace SOArm100::Kinematics::Test
{
class TwistTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
  }

  void TearDown() override
  {
  }
};

TEST_F(TwistTest, ConstructorWithPointOnAxis)
    {
        Eigen::Vector3d axis(0.0, 0.0, 1.0);
        Eigen::Vector3d point_on_axis(0.5, 0.5, 1.5);

        Twist twist(axis, point_on_axis);

        Eigen::Vector3d expected_linear(0.5, -0.5, 0.0);
        ASSERT_TRUE(twist.GetAxis().isApprox(axis.normalized()));
        ASSERT_TRUE(twist.GetLinear().isApprox(expected_linear));
}

TEST_F(TwistTest, ConstructorWithTransform)
    {
        Eigen::Vector3d axis(0.0, 0.0, 1.0);
        Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
        transform(0, 3) = 0.5; // x = 0.5
        transform(1, 3) = 0.5; // y = 0.5
        transform(2, 3) = 0.5; // z = 0.5

        Twist twist(axis, transform);

        Eigen::Vector3d expected_linear(0.5, -0.5, 0.0);
        ASSERT_TRUE(twist.GetAxis().isApprox(axis.normalized()));
        ASSERT_TRUE(twist.GetLinear().isApprox(expected_linear));
}

} // namespace SOArm100::Kinematics::Test
