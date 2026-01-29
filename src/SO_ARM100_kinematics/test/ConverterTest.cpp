#include "Converter.hpp"

#include <gtest/gtest.h>

namespace SOArm100::Kinematics::Test
{
class ConvertTest : public ::testing::Test {
protected:
void SetUp() override
{
}

void TearDown() override
{
}
};

TEST_F( ConvertTest, ToPoseMsgTest )
{
	Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
	matrix( 0, 3 ) = 1.0;
	matrix( 1, 3 ) = 2.0;
	matrix( 2, 3 ) = 3.0;

	geometry_msgs::msg::Pose pose_msg = SOArm100::Kinematics::ToPoseMsg( matrix );

	EXPECT_DOUBLE_EQ( pose_msg.position.x, 1.0 );
	EXPECT_DOUBLE_EQ( pose_msg.position.y, 2.0 );
	EXPECT_DOUBLE_EQ( pose_msg.position.z, 3.0 );
	EXPECT_DOUBLE_EQ( pose_msg.orientation.x, 0.0 );
	EXPECT_DOUBLE_EQ( pose_msg.orientation.y, 0.0 );
	EXPECT_DOUBLE_EQ( pose_msg.orientation.z, 0.0 );
	EXPECT_DOUBLE_EQ( pose_msg.orientation.w, 1.0 );
}
}
