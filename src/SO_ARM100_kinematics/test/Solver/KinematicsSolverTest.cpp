#include "KinematicsSolver.hpp"

#include "RobotModelTestData.hpp"

#include "Model/Joint/JointChain.hpp"
#include "Utils/KinematicsUtils.hpp"

#include <cmath>
#include <gtest/gtest.h>
#include <moveit/robot_model/joint_model_group.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <rclcpp/rclcpp.hpp>
#include <span>

namespace SOArm100::Kinematics::Test
{

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
		Data::GetZYZRevoluteRobotMoveitModel(),
		"arm",
		"base_link",
		{ "end_effector" },
		0.01 );
}

void TearDown() override
{
}

protected:
KinematicsSolver solver_;
};

// ------------------------------------------------------------
// ------------------------------------------------------------

TEST_F( KinematicsSolverTest, Initialize_JointChainInitializationCorrect )
{
	KinematicsSolver solver;
	solver.Initialize(
		Data::GetZYZRevoluteRobotMoveitModel(),
		"arm",
		"base_link",
		{ "end_effector" },
		0.01 );

	Model::JointChain initialized_chain = *solver.GetModel()->GetChain();
	Model::JointChain expected_chain    = Data::GetZYZRevoluteRobotJointChain();

	EXPECT_EQ( expected_chain.GetJointCount(), initialized_chain.GetJointCount() );
	EXPECT_EQ( expected_chain.GetActiveJointCount(), initialized_chain.GetActiveJointCount() );

	auto expected_joints    = expected_chain.GetJoints();
	auto initialized_joints = initialized_chain.GetJoints();

	for ( int i = 0; i < expected_chain.GetJointCount(); i++ )
	{
		auto expected_joint = expected_joints[i];
		auto initialized_joint = initialized_joints[i];
		EXPECT_EQ( expected_joint->GetType(), initialized_joint->GetType() );

		EXPECT_EQ( expected_joint->GetTwist().Omega(), initialized_joint->GetTwist().Omega() );
		EXPECT_EQ( expected_joint->GetTwist().V(), initialized_joint->GetTwist().V() );

		EXPECT_EQ( expected_joint->GetLink().GetJointOrigin(), initialized_joint->GetLink().GetJointOrigin() );
		EXPECT_EQ( expected_joint->GetLink().GetLength(), initialized_joint->GetLink().GetLength() );

		EXPECT_NEAR( expected_joint->GetLimits().Min(), initialized_joint->GetLimits().Min(), 1e-5 );
		EXPECT_NEAR( expected_joint->GetLimits().Max(), initialized_joint->GetLimits().Max(), 1e-5 );
	}
}

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

	auto transform  = Data::GetZYZRevoluteRobotTransform( joint_angles[0], joint_angles[1], joint_angles[2] );
	auto translation = Translation( transform );
	Quaternion quaternion( Rotation( transform ) );

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

	auto transform  = Data::GetZYZRevoluteRobotTransform( joint_angles[0], joint_angles[1], joint_angles[2] );
	auto translation = Translation( transform );
	Quaternion quaternion( Rotation( transform ) );

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

	auto transform  = Data::GetZYZRevoluteRobotTransform( joint_angles[0], joint_angles[1], joint_angles[2] );
	auto translation = Translation( transform );
	Quaternion quaternion( Rotation( transform ) );

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

	auto transform  = Data::GetZYZRevoluteRobotTransform( joint_angles[0], joint_angles[1], joint_angles[2] );
	auto translation = Translation( transform );
	Quaternion quaternion( Rotation( transform ) );

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

} // namespace SOArm100::Kinematics::Test
