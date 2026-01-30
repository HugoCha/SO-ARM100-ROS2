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

TEST_F( KinematicsUtilsTest, SkewMatrixTest )
{
	Eigen::Vector3d vec( 1.0, 2.0, 3.0 );
	Eigen::Matrix3d expected;
	expected << 0, -3.0, 2.0,
	    3.0, 0, -1.0,
	    -2.0, 1.0, 0;

	Eigen::Matrix3d result = SOArm100::Kinematics::SkewMatrix( vec );

	ASSERT_TRUE( result.isApprox( expected ) );
}

}
