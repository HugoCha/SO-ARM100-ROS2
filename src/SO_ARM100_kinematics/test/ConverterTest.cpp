#include "Converter.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtest/gtest.h>

namespace SOArm100::Kinematics::Test
{

// ------------------------------------------------------------
// ------------------------------------------------------------

class ConvertTest : public ::testing::Test
{
protected:
void SetUp() override {
}
void TearDown() override {
}
};

// ------------------------------------------------------------
// ------------------------------------------------------------

TEST_F( ConvertTest, ToVecXdTest )
{
	std::vector< double > data{ 1.0, 2.0, 3.0 };

	VecXd vec = SOArm100::Kinematics::ToVecXd( data );

	ASSERT_EQ( vec.size(), 3 );
	EXPECT_DOUBLE_EQ( vec( 0 ), 1.0 );
	EXPECT_DOUBLE_EQ( vec( 1 ), 2.0 );
	EXPECT_DOUBLE_EQ( vec( 2 ), 3.0 );
}

// ------------------------------------------------------------

TEST_F( ConvertTest, ToStdVectorTest )
{
	VecXd vec( 3 );
	vec << 4.0, 5.0, 6.0;

	std::vector< double > result =
		SOArm100::Kinematics::ToStdVector( vec );

	ASSERT_EQ( result.size(), 3 );
	EXPECT_DOUBLE_EQ( result[0], 4.0 );
	EXPECT_DOUBLE_EQ( result[1], 5.0 );
	EXPECT_DOUBLE_EQ( result[2], 6.0 );
}

// ------------------------------------------------------------

TEST_F( ConvertTest, ToMat4dTest )
{
	geometry_msgs::msg::Pose pose;
	pose.position.x = 1.0;
	pose.position.y = 2.0;
	pose.position.z = 3.0;

	// 90° rotation around Z
	Eigen::Quaterniond q( Eigen::AngleAxisd( M_PI / 2.0, Eigen::Vector3d::UnitZ() ) );
	pose.orientation.x = q.x();
	pose.orientation.y = q.y();
	pose.orientation.z = q.z();
	pose.orientation.w = q.w();

	Mat4d mat = SOArm100::Kinematics::ToMat4d( pose );

	EXPECT_DOUBLE_EQ( mat( 0, 3 ), 1.0 );
	EXPECT_DOUBLE_EQ( mat( 1, 3 ), 2.0 );
	EXPECT_DOUBLE_EQ( mat( 2, 3 ), 3.0 );

	Eigen::Matrix3d R_expected = q.normalized().toRotationMatrix();
	Eigen::Matrix3d R_actual = mat.block< 3, 3 >( 0, 0 );

	EXPECT_TRUE( R_expected.isApprox( R_actual, 1e-12 ) )
	    << "Expected = " << std::endl << R_expected << std::endl
	    << "Actual = "   << std::endl << R_actual << std::endl;
}

// ------------------------------------------------------------

TEST_F( ConvertTest, ToPoseMsgTest )
{
	Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
	matrix( 0, 3 ) = 1.0;
	matrix( 1, 3 ) = 2.0;
	matrix( 2, 3 ) = 3.0;

	geometry_msgs::msg::Pose pose_msg =
		SOArm100::Kinematics::ToPoseMsg( matrix );

	EXPECT_DOUBLE_EQ( pose_msg.position.x, 1.0 );
	EXPECT_DOUBLE_EQ( pose_msg.position.y, 2.0 );
	EXPECT_DOUBLE_EQ( pose_msg.position.z, 3.0 );
	EXPECT_DOUBLE_EQ( pose_msg.orientation.x, 0.0 );
	EXPECT_DOUBLE_EQ( pose_msg.orientation.y, 0.0 );
	EXPECT_DOUBLE_EQ( pose_msg.orientation.z, 0.0 );
	EXPECT_DOUBLE_EQ( pose_msg.orientation.w, 1.0 );
}

// ------------------------------------------------------------

TEST_F( ConvertTest, PoseRoundTripTest )
{
	geometry_msgs::msg::Pose input_pose;
	input_pose.position.x = -0.5;
	input_pose.position.y = 1.2;
	input_pose.position.z = 0.8;

	Eigen::Quaterniond q(
		Eigen::AngleAxisd( 0.3, Eigen::Vector3d::UnitX() ) *
		Eigen::AngleAxisd( -0.2, Eigen::Vector3d::UnitY() ) *
		Eigen::AngleAxisd( 1.0, Eigen::Vector3d::UnitZ() ) );

	input_pose.orientation.x = q.x();
	input_pose.orientation.y = q.y();
	input_pose.orientation.z = q.z();
	input_pose.orientation.w = q.w();

	Mat4d mat = SOArm100::Kinematics::ToMat4d( input_pose );
	geometry_msgs::msg::Pose output_pose =
		SOArm100::Kinematics::ToPoseMsg( mat );

	EXPECT_NEAR( output_pose.position.x, input_pose.position.x, 1e-12 );
	EXPECT_NEAR( output_pose.position.y, input_pose.position.y, 1e-12 );
	EXPECT_NEAR( output_pose.position.z, input_pose.position.z, 1e-12 );

	Eigen::Quaterniond q_out(
		output_pose.orientation.w,
		output_pose.orientation.x,
		output_pose.orientation.y,
		output_pose.orientation.z );

	EXPECT_TRUE( q.normalized().isApprox( q_out.normalized(), 1e-12 ) );
}

// ------------------------------------------------------------

}