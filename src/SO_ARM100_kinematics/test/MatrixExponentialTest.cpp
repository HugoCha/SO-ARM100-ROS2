#include "MatrixExponential.hpp"

#include <gtest/gtest.h>

namespace SOArm100::Kinematics::Test
{
class MatrixExponentialTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
  }

  void TearDown() override
  {
  }
};

TEST_F(MatrixExponentialTest, ComputeTest)
    {
        Eigen::Vector3d axis(0.0, 0.0, 1.0);
        Eigen::Vector3d point_on_axis(0.0, 0.0, 0.0);
        Twist twist(axis, point_on_axis);
        double theta = M_PI / 2; // 90 degrees

        MatrixExponential matrix_exponential(twist, theta);
        Eigen::Matrix4d result = matrix_exponential.Compute();

        Eigen::Matrix4d expected = Eigen::Matrix4d::Identity();
        expected(0, 0) = 0.0;
        expected(0, 1) = -1.0;
        expected(1, 0) = 1.0;
        expected(1, 1) = 0.0;

        ASSERT_TRUE(result.isApprox(expected));
}

TEST_F(MatrixExponentialTest, OperatorMultiplyTest)
    {
        Eigen::Vector3d axis(0.0, 0.0, 1.0);
        Eigen::Vector3d point_on_axis(0.0, 0.0, 0.0);
        Twist twist(axis, point_on_axis);
        double theta1 = M_PI / 2; // 90 degrees
        double theta2 = M_PI / 2; // 90 degrees

        MatrixExponential matrix_exponential1(twist, theta1);
        MatrixExponential matrix_exponential2(twist, theta2);
        Eigen::Matrix4d result = matrix_exponential1 * matrix_exponential2;

        Eigen::Matrix4d expected = Eigen::Matrix4d::Identity();
        expected(0, 0) = -1.0;
        expected(0, 1) = 0.0;
        expected(1, 0) = 0.0;
        expected(1, 1) = -1.0;

        ASSERT_TRUE(result.isApprox(expected));
}

TEST_F(MatrixExponentialTest, OperatorMatrixMultiplyTest)
{
        Eigen::Vector3d axis(0.0, 0.0, 1.0);
        Eigen::Vector3d point_on_axis(0.0, 0.0, 0.0);
        Twist twist(axis, point_on_axis);
        double theta = M_PI / 2; // 90 degrees

        MatrixExponential matrix_exponential(twist, theta);
        Eigen::Matrix4d input_matrix = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d result = matrix_exponential * input_matrix;

        Eigen::Matrix4d expected = Eigen::Matrix4d::Identity();
        expected(0, 0) = 0.0;
        expected(0, 1) = -1.0;
        expected(1, 0) = 1.0;
        expected(1, 1) = 0.0;

        ASSERT_TRUE(result.isApprox(expected));
}
} // namespace SOArm100::Kinematics::Test
