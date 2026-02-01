#include "KinematicsSolver.hpp"

#include "RobotModelTestData.hpp"

#include <cmath>
#include <gtest/gtest.h>
#include <moveit/robot_model/joint_model_group.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <rclcpp/rclcpp.hpp>
#include <span>

namespace SOArm100::Kinematics::Test
{
class DummyKinematicsSolver : public KinematicsSolver
{
public:
bool InverseKinematic(
	const geometry_msgs::msg::Pose& target_pose,
	const std::span< const double >& initial_joints,
	std::vector< double >& joint_angles ) const override
{
	return false;
}

bool CheckLimitsExposed( const std::vector< double >& joint_angles )
{
	return CheckLimits( joint_angles );
}

const moveit::core::JointModelGroup* GetJointModelGroup()
{
	return joint_model_;
}
};

/**
 * Test for a robot model with:
 * - 3 revolute joints (rotation around z-axis, y-axis, z-axis)
 */
class RevoluteOnlyKinematicsSolverTest : public ::testing::Test
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

TEST_F( RevoluteOnlyKinematicsSolverTest, ForwardKinematicsHomePosition )
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

/**
 * Test ForwardKinematics with revolute joint rotation (z-axis)
 */
TEST_F( RevoluteOnlyKinematicsSolverTest, ForwardKinematicsWithRevoluteRotationZ )
{
	// Rotate joint_1 (z-axis) by 90 degrees (pi/2 radians)
	std::vector< double > joint_angles = { 1.5708, 0.0, 0.0 };     // pi/2 ≈ 1.5708
	geometry_msgs::msg::Pose end_effector_pose;

	bool result = solver_.ForwardKinematic( joint_angles, end_effector_pose );

	ASSERT_TRUE( result ) << "ForwardKinematic should succeed with z-axis rotation";

	// After 90 degree rotation around z-axis at joint_1:
	// The 0.5m offset in x becomes 0.5m in y, then the second link's 0.5m x becomes 0.5m y
	// Result: (0.0, 1.0, 0.0) approximately
	EXPECT_NEAR( end_effector_pose.position.x, 0.0, 0.01 );
	EXPECT_NEAR( end_effector_pose.position.y, 1.0, 0.01 );
	EXPECT_NEAR( end_effector_pose.position.z, 0.0, 0.01 );
}

/**
 * Test ForwardKinematics with revolute joint rotation (y-axis)
 */
TEST_F( RevoluteOnlyKinematicsSolverTest, ForwardKinematicsWithRevoluteRotationY )
{
	// Rotate joint_2 (y-axis) by 90 degrees (pi/2 radians)
	std::vector< double > joint_angles = { 0.0, 1.5708, 0.0 };     // pi/2 ≈ 1.5708
	geometry_msgs::msg::Pose end_effector_pose;

	bool result = solver_.ForwardKinematic( joint_angles, end_effector_pose );

	ASSERT_TRUE( result ) << "ForwardKinematic should succeed with y-axis rotation";

	// After 90 degree rotation around y-axis at link_1 (right-hand rule):
	// Positive x becomes negative z. The 0.5m offset in x becomes -0.5m in z,
	// then the second link's 0.5m x becomes -0.5m z.
	// Result: (0.0, 0.0, -1.0) approximately
	EXPECT_NEAR( end_effector_pose.position.x, 0.0, 0.01 );
	EXPECT_NEAR( end_effector_pose.position.y, 0.0, 0.01 );
	EXPECT_NEAR( end_effector_pose.position.z, -1.0, 0.01 );
}

/**
 * Test ForwardKinematics with combined joint movements
 */
TEST_F( RevoluteOnlyKinematicsSolverTest, ForwardKinematicsWithCombinedMovements )
{
	// joint_1: 45 degrees, joint_2: 30 degrees, joint_3: 0
	std::vector< double > joint_angles = { 0.7854, 0.5236, 0.0 };     // pi/4 ≈ 0.7854, pi/6 ≈ 0.5236
	geometry_msgs::msg::Pose end_effector_pose;

	bool result = solver_.ForwardKinematic( joint_angles, end_effector_pose );

	ASSERT_TRUE( result ) << "ForwardKinematic should succeed with combined movements";

	// Verify that the pose was computed (values should be within reasonable bounds)
	EXPECT_GE( end_effector_pose.position.x, -2.0 );
	EXPECT_LE( end_effector_pose.position.x, 2.0 );
	EXPECT_GE( end_effector_pose.position.y, -2.0 );
	EXPECT_LE( end_effector_pose.position.y, 2.0 );
	EXPECT_GE( end_effector_pose.position.z, -2.0 );
	EXPECT_LE( end_effector_pose.position.z, 2.0 );
}

/**
 * Test ForwardKinematics with all joints in motion
 */
TEST_F( RevoluteOnlyKinematicsSolverTest, ForwardKinematicsAllJointsInMotion )
{
	// All joints moving: rev_z, rev_y, rev_z
	std::vector< double > joint_angles = { 1.5708, 0.7854, 0.3927 };     // Various angles
	geometry_msgs::msg::Pose end_effector_pose;

	bool result = solver_.ForwardKinematic( joint_angles, end_effector_pose );

	ASSERT_TRUE( result ) << "ForwardKinematic should succeed with all joints in motion";

	// Just verify output is reasonable (within robot workspace bounds)
	EXPECT_GE( end_effector_pose.position.x, -2.0 );
	EXPECT_LE( end_effector_pose.position.x, 2.0 );
	EXPECT_GE( end_effector_pose.position.y, -2.0 );
	EXPECT_LE( end_effector_pose.position.y, 2.0 );
	EXPECT_GE( end_effector_pose.position.z, -2.0 );
	EXPECT_LE( end_effector_pose.position.z, 2.0 );

	// Verify orientation is normalized (unit quaternion)
	double quat_magnitude = std::sqrt(
		end_effector_pose.orientation.x * end_effector_pose.orientation.x +
		end_effector_pose.orientation.y * end_effector_pose.orientation.y +
		end_effector_pose.orientation.z * end_effector_pose.orientation.z +
		end_effector_pose.orientation.w * end_effector_pose.orientation.w );
	EXPECT_NEAR( quat_magnitude, 1.0, 0.01 ) << "Quaternion should be normalized";
}

TEST_F( RevoluteOnlyKinematicsSolverTest, CheckLimitsValid )
{
	// All joints within [-pi, pi]
	std::vector< double > joint_angles = { 0.0, 0.5, -1.0 };

	bool result = solver_.CheckLimitsExposed( joint_angles );

	EXPECT_TRUE( result ) << "Joint angles within limits should be valid";
}

TEST_F( RevoluteOnlyKinematicsSolverTest, CheckLimitsBelowLowerLimit )
{
	// joint_1 lower limit is approx -pi
	std::vector< double > joint_angles = { -4.0, 0.0, 0.0 };

	bool result = solver_.CheckLimitsExposed( joint_angles );

	EXPECT_FALSE( result ) << "Joint angle below lower limit should be invalid";
}
TEST_F( RevoluteOnlyKinematicsSolverTest, CheckLimitsAboveUpperLimit )
{
	// joint_2 upper limit is approx +pi
	std::vector< double > joint_angles = { 0.0, 4.0, 0.0 };

	bool result = solver_.CheckLimitsExposed( joint_angles );

	EXPECT_FALSE( result ) << "Joint angle above upper limit should be invalid";
}

TEST_F( RevoluteOnlyKinematicsSolverTest, CheckLimitsOneJointInvalid )
{
	std::vector< double > joint_angles = { 0.0, 0.0, 10.0 };

	bool result = solver_.CheckLimitsExposed( joint_angles );

	EXPECT_FALSE( result ) << "Single joint violation should invalidate the whole configuration";
}

} // namespace SOArm100::Kinematics::Test
