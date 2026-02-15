#include "KinematicsSolver.hpp"

#include "Utils/KinematicsUtils.hpp"
#include "RobotModelTestData.hpp"

#include <cmath>
#include <gtest/gtest.h>
#include <moveit/robot_model/joint_model_group.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <rclcpp/rclcpp.hpp>
#include <span>

namespace SOArm100::Kinematics::Test
{

// ------------------------------------------------------------
// ------------------------------------------------------------

class DummyKinematicsSolver : public KinematicsSolver
{
public:
bool InverseKinematic(
	const Mat4d& target_pose,
	const std::span< const double >& initial_joints,
	VecXd& joint_angles ) const override
{
	return false;
}

bool CheckLimitsExposed( const std::vector< double >& joint_angles )
{
	return CheckLimits( joint_angles );
}
};

// ------------------------------------------------------------
// ------------------------------------------------------------

class KinematicsSolverTest : public ::testing::Test
{
protected:
void SetUp() override
{
	// Initialize ROS context for logger support
	if ( !rclcpp::ok() )
	{
		int argc = 0;
		char** argv = nullptr;
		rclcpp::init( argc, argv );
	}

	// Initialize solver with the robot model
	solver_.Initialize(
		Data::GetRevoluteOnlyRobot(),
		"arm",
		"base_link",
		{ "end_effector" },
		0.01 );
}

void TearDown() override
{
}

protected:
DummyKinematicsSolver solver_;
};

// ------------------------------------------------------------
// ------------------------------------------------------------

TEST_F( KinematicsSolverTest, ForwardKinematicsHomePosition )
{
	std::vector< double > joint_angles = { 0.0, 0.0, 0.0 };
	geometry_msgs::msg::Pose end_effector_pose;

	bool result = solver_.ForwardKinematic( joint_angles, end_effector_pose );

	ASSERT_TRUE( result ) << "ForwardKinematic should succeed with home position";

	// At home position, end effector should be at:
	// base_link + link_1 (0.5m offset along x) + link_2 (0.5m offset along x)
	// = (1.0, 0, 0) in base frame
	EXPECT_NEAR( end_effector_pose.position.x, 1.0, 0.01 );
	EXPECT_NEAR( end_effector_pose.position.y, 0.0, 0.01 );
	EXPECT_NEAR( end_effector_pose.position.z, 0.0, 0.01 );

	// At home position, orientation should be identity (no rotations)
	EXPECT_NEAR( end_effector_pose.orientation.w, 1.0, 0.01 );
	EXPECT_NEAR( end_effector_pose.orientation.x, 0.0, 0.01 );
	EXPECT_NEAR( end_effector_pose.orientation.y, 0.0, 0.01 );
	EXPECT_NEAR( end_effector_pose.orientation.z, 0.0, 0.01 );
}

// ------------------------------------------------------------

TEST_F( KinematicsSolverTest, ForwardKinematicsWithRevoluteRotationZ )
{
	// Rotate joint_1 (z-axis) by 90 degrees (pi/2 radians)
	std::vector< double > joint_angles = { 1.5708 / 2, 0.0, 0.0 };     // pi/2 ≈ 1.5708
	geometry_msgs::msg::Pose end_effector_pose;

	bool result = solver_.ForwardKinematic( joint_angles, end_effector_pose );

	ASSERT_TRUE( result ) << "ForwardKinematic should succeed with z-axis rotation";

	auto transform  = Data::GetRevoluteOnlyRobotTransform( joint_angles[0], joint_angles[1], joint_angles[2] );
	auto translation = Translation( transform );
	Eigen::Quaterniond quaternion( Rotation( transform ) );

	// Verify that the pose was computed (values should be within reasonable bounds)
	EXPECT_NEAR( end_effector_pose.position.x, translation.x(), 1e-3 );
	EXPECT_NEAR( end_effector_pose.position.y, translation.y(), 1e-3 );
	EXPECT_NEAR( end_effector_pose.position.z, translation.z(), 1e-3 );

	EXPECT_NEAR( end_effector_pose.orientation.w, quaternion.w(), 1e-3 );
	EXPECT_NEAR( end_effector_pose.orientation.x, quaternion.x(), 1e-3 );
	EXPECT_NEAR( end_effector_pose.orientation.y, quaternion.y(), 1e-3 );
	EXPECT_NEAR( end_effector_pose.orientation.z, quaternion.z(), 1e-3 );
}

// ------------------------------------------------------------

TEST_F( KinematicsSolverTest, ForwardKinematicsWithRevoluteRotationY )
{
	// Rotate joint_2 (y-axis) by 90 degrees (pi/2 radians)
	std::vector< double > joint_angles = { 0.0, 1.5708, 0.0 };     // pi/2 ≈ 1.5708
	geometry_msgs::msg::Pose end_effector_pose;

	bool result = solver_.ForwardKinematic( joint_angles, end_effector_pose );

	ASSERT_TRUE( result ) << "ForwardKinematic should succeed with y-axis rotation";

	auto transform  = Data::GetRevoluteOnlyRobotTransform( joint_angles[0], joint_angles[1], joint_angles[2] );
	auto translation = Translation( transform );
	Eigen::Quaterniond quaternion( Rotation( transform ) );

	// Verify that the pose was computed (values should be within reasonable bounds)
	EXPECT_NEAR( end_effector_pose.position.x, translation.x(), 1e-3 );
	EXPECT_NEAR( end_effector_pose.position.y, translation.y(), 1e-3 );
	EXPECT_NEAR( end_effector_pose.position.z, translation.z(), 1e-3 );

	EXPECT_NEAR( end_effector_pose.orientation.w, quaternion.w(), 1e-3 );
	EXPECT_NEAR( end_effector_pose.orientation.x, quaternion.x(), 1e-3 );
	EXPECT_NEAR( end_effector_pose.orientation.y, quaternion.y(), 1e-3 );
	EXPECT_NEAR( end_effector_pose.orientation.z, quaternion.z(), 1e-3 );
}

// ------------------------------------------------------------

TEST_F( KinematicsSolverTest, ForwardKinematicsWithCombinedMovements )
{
	// joint_1: 45 degrees, joint_2: 30 degrees, joint_3: 0
	std::vector< double > joint_angles = { 0.7854, 0.5236, 0.0 };     // pi/4 ≈ 0.7854, pi/6 ≈ 0.5236
	geometry_msgs::msg::Pose end_effector_pose;

	bool result = solver_.ForwardKinematic( joint_angles, end_effector_pose );

	ASSERT_TRUE( result ) << "ForwardKinematic should succeed with combined movements";

	auto transform  = Data::GetRevoluteOnlyRobotTransform( joint_angles[0], joint_angles[1], joint_angles[2] );
	auto translation = Translation( transform );
	Eigen::Quaterniond quaternion( Rotation( transform ) );

	// Verify that the pose was computed (values should be within reasonable bounds)
	EXPECT_NEAR( end_effector_pose.position.x, translation.x(), 1e-3 );
	EXPECT_NEAR( end_effector_pose.position.y, translation.y(), 1e-3 );
	EXPECT_NEAR( end_effector_pose.position.z, translation.z(), 1e-3 );

	EXPECT_NEAR( end_effector_pose.orientation.w, quaternion.w(), 1e-3 );
	EXPECT_NEAR( end_effector_pose.orientation.x, quaternion.x(), 1e-3 );
	EXPECT_NEAR( end_effector_pose.orientation.y, quaternion.y(), 1e-3 );
	EXPECT_NEAR( end_effector_pose.orientation.z, quaternion.z(), 1e-3 );
}

// ------------------------------------------------------------

TEST_F( KinematicsSolverTest, ForwardKinematicsAllJointsInMotion )
{
	// All joints moving: rev_z, rev_y, rev_z
	std::vector< double > joint_angles = { 1.5708, 0.7854, 0.3927 };     // Various angles
	geometry_msgs::msg::Pose end_effector_pose;

	bool result = solver_.ForwardKinematic( joint_angles, end_effector_pose );

	ASSERT_TRUE( result ) << "ForwardKinematic should succeed with all joints in motion";

	auto transform  = Data::GetRevoluteOnlyRobotTransform( joint_angles[0], joint_angles[1], joint_angles[2] );
	auto translation = Translation( transform );
	Eigen::Quaterniond quaternion( Rotation( transform ) );

	std::cout << "Expected = \n" << transform << std::endl;
	// Verify that the pose was computed (values should be within reasonable bounds)
	EXPECT_NEAR( end_effector_pose.position.x, translation.x(), 1e-3 );
	EXPECT_NEAR( end_effector_pose.position.y, translation.y(), 1e-3 );
	EXPECT_NEAR( end_effector_pose.position.z, translation.z(), 1e-3 );

	EXPECT_NEAR( end_effector_pose.orientation.w, quaternion.w(), 1e-3 );
	EXPECT_NEAR( end_effector_pose.orientation.x, quaternion.x(), 1e-3 );
	EXPECT_NEAR( end_effector_pose.orientation.y, quaternion.y(), 1e-3 );
	EXPECT_NEAR( end_effector_pose.orientation.z, quaternion.z(), 1e-3 );
}

// ------------------------------------------------------------

TEST_F( KinematicsSolverTest, CheckLimitsValid )
{
	// All joints within [-pi, pi]
	std::vector< double > joint_angles = { 0.0, 0.5, -1.0 };

	bool result = solver_.CheckLimitsExposed( joint_angles );

	EXPECT_TRUE( result ) << "Joint angles within limits should be valid";
}

// ------------------------------------------------------------

TEST_F( KinematicsSolverTest, CheckLimitsBelowLowerLimit )
{
	// joint_1 lower limit is approx -pi
	std::vector< double > joint_angles = { -4.0, 0.0, 0.0 };

	bool result = solver_.CheckLimitsExposed( joint_angles );

	EXPECT_FALSE( result ) << "Joint angle below lower limit should be invalid";
}

// ------------------------------------------------------------

TEST_F( KinematicsSolverTest, CheckLimitsAboveUpperLimit )
{
	// joint_2 upper limit is approx +pi
	std::vector< double > joint_angles = { 0.0, 4.0, 0.0 };

	bool result = solver_.CheckLimitsExposed( joint_angles );

	EXPECT_FALSE( result ) << "Joint angle above upper limit should be invalid";
}

// ------------------------------------------------------------

TEST_F( KinematicsSolverTest, CheckLimitsOneJointInvalid )
{
	std::vector< double > joint_angles = { 0.0, 0.0, 10.0 };

	bool result = solver_.CheckLimitsExposed( joint_angles );

	EXPECT_FALSE( result ) << "Single joint violation should invalidate the whole configuration";
}

// ------------------------------------------------------------

} // namespace SOArm100::Kinematics::Test
