#include "HybridSolver/WristSolver.hpp"

#include "Global.hpp"
#include "HybridSolver/WristModel.hpp"
#include "Joint/JointChain.hpp"
#include "RobotModelTestData.hpp"
#include "Utils/KinematicsUtils.hpp"
#include "SolverResult.hpp"

#include <gtest/gtest.h>
#include <memory>

namespace SOArm100::Kinematics::Test
{

// ------------------------------------------------------------
// ------------------------------------------------------------

class WristSolverTest : public ::testing::Test
{
protected:
void SetUp() override
{
}

void TearDown() override
{
}
};

// ------------------------------------------------------------
// ------------------------------------------------------------

TEST_F( WristSolverTest, ComputeWristCenter )
{
	auto joint_chain = std::make_shared< const JointChain >( Data::GetRevoluteOnlyRobotJointChain() );

	// Create a wrist model for the last 3 joints
	WristModel wrist_model;
	wrist_model.type = WristType::Revolute3;
	wrist_model.active_joint_start = joint_chain->GetActiveJointCount() - 3;
	wrist_model.active_joint_count = 3;
	wrist_model.center_at_home = Vec3d( 0, 0, 0 );
	wrist_model.tcp_in_wrist_at_home = Mat4d::Identity();
	wrist_model.tcp_in_wrist_at_home_inv = Mat4d::Identity();

	auto home = std::make_shared< const Mat4d >( wrist_model.tcp_in_wrist_at_home );
	// Create a target transform
	Mat4d target = Mat4d::Identity();
	target.block< 3, 3 >( 0, 0 ) = Eigen::AngleAxisd( M_PI / 4, Vec3d( 0, 0, 1 ) ).toRotationMatrix();
	target.block< 3, 1 >( 0, 3 ) = Vec3d( 1.0, 0.0, 0.0 );

	// Compute the wrist center
	Vec3d wrist_center;
	WristSolver solver( joint_chain, home, wrist_model );
	solver.ComputeWristCenter( target, wrist_center );

	auto target_translation = target.block< 3, 1 >( 0, 3 );
	// The wrist center should be the target transform multiplied by the inverse of the TCP transform
	// Since the TCP transform is identity, the wrist center should be the same as the target
	EXPECT_TRUE( target_translation.isApprox( wrist_center, error_tolerance ) )
	    << "Wrist center should match target when TCP transform is identity";
}

// ------------------------------------------------------------

TEST_F( WristSolverTest, SolveRevolute1 )
{
	// Create a joint chain with a single revolute joint
	auto single_joint_chain = std::make_shared< JointChain >( 1 );
	single_joint_chain->Add(
		Twist( Vec3d( 0, 0, 1 ), Vec3d( 0, 0, 0 ) ),
		Link( Mat4d::Identity() ),
		Limits( -M_PI, M_PI )
		);

	// Create a wrist model for the single joint
	WristModel single_wrist_model;
	single_wrist_model.type = WristType::Revolute1;
	single_wrist_model.active_joint_start = 0;
	single_wrist_model.active_joint_count = 1;
	single_wrist_model.center_at_home = Vec3d( 0, 0, 0 );
	single_wrist_model.tcp_in_wrist_at_home = Mat4d::Identity();
	single_wrist_model.tcp_in_wrist_at_home_inv = Mat4d::Identity();
	auto home = std::make_shared< const Mat4d >( single_wrist_model.tcp_in_wrist_at_home );

	// Initialize a solver for the single joint
	WristSolver single_solver( single_joint_chain, home, single_wrist_model );

	// Test with a target rotation around the Z-axis
	Mat4d R_target = Mat4d::Identity();
	R_target.block< 3, 3 >( 0, 0 ) = Eigen::AngleAxisd( M_PI / 4, Vec3d( 0, 0, 1 ) ).toRotationMatrix();

	// Solve IK
	std::vector< double > seeds = { 0.0 };
	SolverResult result = single_solver.IK( R_target, seeds, 0 );

	// Check that the solution is valid
	EXPECT_EQ( result.state, SolverState::Success ) << "IK should succeed for reachable target";
	EXPECT_NEAR( result.joints[0], M_PI / 4, 1e-6 ) << "Solution should match target angle";

	// Verify the solution by checking the resulting rotation
	Mat3d R_result = Eigen::AngleAxisd( result.joints[0], Vec3d( 0, 0, 1 ) ).toRotationMatrix();
	EXPECT_TRUE( R_result.isApprox( R_target.block< 3, 3 >( 0, 0 ), rotation_tolerance ) ) << "Resulting rotation should match target";
}

// ------------------------------------------------------------

TEST_F( WristSolverTest, SolveRevolute1_Unreachable )
{
	// Create a joint chain with a single revolute joint
	auto single_joint_chain = std::make_shared< JointChain >( 1 );
	single_joint_chain->Add(
		Twist( Vec3d( 0, 0, 1 ), Vec3d( 0, 0, 0 ) ),
		Link( Mat4d::Identity() ),
		Limits( -M_PI, M_PI )
		);

	// Create a wrist model for the single joint
	WristModel single_wrist_model;
	single_wrist_model.type = WristType::Revolute1;
	single_wrist_model.active_joint_start = 0;
	single_wrist_model.active_joint_count = 1;
	single_wrist_model.center_at_home = Vec3d( 0, 0, 0 );
	single_wrist_model.tcp_in_wrist_at_home = Mat4d::Identity();
	single_wrist_model.tcp_in_wrist_at_home_inv = Mat4d::Identity();
	auto home = std::make_shared< const Mat4d >( single_wrist_model.tcp_in_wrist_at_home );

	// Initialize a solver for the single joint
	WristSolver single_solver( single_joint_chain, home, single_wrist_model );

	// Test with a target rotation around the X-axis (unreachable for a Z-axis joint)
	Mat4d R_target = Mat4d::Identity();
	R_target.block< 3, 3 >( 0, 0 ) = Eigen::AngleAxisd( M_PI / 4, Vec3d( 1, 0, 0 ) ).toRotationMatrix();

	// Solve IK
	std::vector< double > seeds = { 0.0 };
	SolverResult result = single_solver.IK( R_target, seeds, 0 );

	// Check that the solution is unreachable
	EXPECT_EQ( result.state, SolverState::Unreachable ) << "IK should fail for unreachable target";
}

// ------------------------------------------------------------

TEST_F( WristSolverTest, SolveRevolute2 )
{
	// Create a joint chain with two revolute joints
	auto two_joint_chain = std::make_shared< JointChain >( 2 );
	two_joint_chain->Add(
		Twist( Vec3d( 0, 0, 1 ), Vec3d( 0, 0, 0 ) ),
		Link( Mat4d::Identity() ),
		Limits( -M_PI, M_PI )
		);
	two_joint_chain->Add(
		Twist( Vec3d( 0, 1, 0 ), Vec3d( 0, 0, 0 ) ),
		Link( Mat4d::Identity() ),
		Limits( -M_PI, M_PI )
		);

	// Create a wrist model for the two joints
	WristModel two_wrist_model;
	two_wrist_model.type = WristType::Revolute2;
	two_wrist_model.active_joint_start = 0;
	two_wrist_model.active_joint_count = 2;
	two_wrist_model.center_at_home = Vec3d( 0, 0, 0 );
	two_wrist_model.tcp_in_wrist_at_home = Mat4d::Identity();
	two_wrist_model.tcp_in_wrist_at_home_inv = Mat4d::Identity();
	auto home = std::make_shared< const Mat4d >( two_wrist_model.tcp_in_wrist_at_home );

	// Initialize a solver for the two joints
	WristSolver two_solver( two_joint_chain, home, two_wrist_model );

	// Test with a target rotation
	Mat4d R_target;
	VecXd initial_joints( 2 );
	initial_joints << M_PI / 3, M_PI / 6;
	POE( *two_joint_chain, Mat4d::Identity(), initial_joints, R_target );

	// Solve IK
	std::vector< double > seeds = { 0.0, 0.0 };
	SolverResult result = two_solver.IK( R_target, seeds, 0 );

	// Check that the solution is valid
	EXPECT_EQ( result.state, SolverState::Success ) << "IK should succeed for reachable target";

	// Verify the solution by checking the resulting rotation
	Mat4d R_result;
	POE( *two_joint_chain, Mat4d::Identity(), result.joints, R_result );

	EXPECT_TRUE( IsApprox( R_target, R_result ) )
	    << "Initial joints: { " << initial_joints.transpose() << " }"  << std::endl
	    << "Expected Mat\n" << R_target  << std::endl
	    << "Result joint: { " << result.joints.transpose() << " }" << std::endl
	    << "Result Mat\n" << R_result << std::endl;
}

// ------------------------------------------------------------

TEST_F( WristSolverTest, SolveRevolute2_Unreachable )
{
	// Create a joint chain with two parallel revolute joints
	auto two_joint_chain = std::make_shared< JointChain >( 2 );
	two_joint_chain->Add(
		Twist( Vec3d( 0, 0, 1 ), Vec3d( 0, 0, 0 ) ),
		Link( Mat4d::Identity() ),
		Limits( -M_PI, M_PI )
		);
	two_joint_chain->Add(
		Twist( Vec3d( 0, 0, 1 ), Vec3d( 0, 0, 1 ) ),  // Parallel to the first joint
		Link( Mat4d::Identity() ),
		Limits( -M_PI, M_PI )
		);

	// Create a wrist model for the two joints
	WristModel two_wrist_model;
	two_wrist_model.type = WristType::Revolute2;
	two_wrist_model.active_joint_start = 0;
	two_wrist_model.active_joint_count = 2;
	two_wrist_model.center_at_home = Vec3d( 0, 0, 0 );
	two_wrist_model.tcp_in_wrist_at_home = Mat4d::Identity();
	two_wrist_model.tcp_in_wrist_at_home_inv = Mat4d::Identity();
	auto home = std::make_shared< const Mat4d >( two_wrist_model.tcp_in_wrist_at_home );

	// Initialize a solver for the two joints
	WristSolver two_solver( two_joint_chain, home, two_wrist_model );

	// Test with a target rotation
	Mat4d R_target = Mat4d::Identity();
	R_target.block< 3, 3 >( 0, 0 ) = Eigen::AngleAxisd( M_PI / 4, Vec3d( 0, 0, 1 ) ).toRotationMatrix() *
	                                 Eigen::AngleAxisd( M_PI / 6, Vec3d( 0, 1, 0 ) ).toRotationMatrix();

	// Solve IK
	std::vector< double > seeds = { 0.0, 0.0 };
	SolverResult result = two_solver.IK( R_target, seeds, 0 );

	// Check that the solution is unreachable (because the axes are parallel)
	EXPECT_EQ( result.state, SolverState::Unreachable ) << "IK should fail for parallel axes";
}

// ------------------------------------------------------------

TEST_F( WristSolverTest, SolveRevolute3 )
{
	// Create a joint chain with three revolute joints (spherical wrist)
	auto three_joint_chain = std::make_shared< JointChain >( 3 );
	three_joint_chain->Add(
		Twist( Vec3d( 0, 0, 1 ), Vec3d( 0, 0, 0 ) ),
		Link( Mat4d::Identity() ),
		Limits( -M_PI, M_PI )
		);
	three_joint_chain->Add(
		Twist( Vec3d( 0, 1, 0 ), Vec3d( 0, 0, 0 ) ),
		Link( Mat4d::Identity() ),
		Limits( -M_PI, M_PI )
		);
	three_joint_chain->Add(
		Twist( Vec3d( 1, 0, 0 ), Vec3d( 0, 0, 0 ) ),
		Link( Mat4d::Identity() ),
		Limits( -M_PI, M_PI )
		);

	// Create a wrist model for the three joints
	WristModel three_wrist_model;
	three_wrist_model.type = WristType::Revolute3;
	three_wrist_model.active_joint_start = 0;
	three_wrist_model.active_joint_count = 3;
	three_wrist_model.center_at_home = Vec3d( 0, 0, 0 );
	three_wrist_model.tcp_in_wrist_at_home = Mat4d::Identity();
	three_wrist_model.tcp_in_wrist_at_home_inv = Mat4d::Identity();
	auto home = std::make_shared< const Mat4d >( three_wrist_model.tcp_in_wrist_at_home );

	// Initialize a solver for the three joints
	WristSolver three_solver( three_joint_chain, home, three_wrist_model );

	// Test with a target rotation
	Mat4d R_target;
	VecXd initial_joints( 3 );
	initial_joints << M_PI / 4, M_PI / 6, M_PI / 8;
	POE( *three_joint_chain, Mat4d::Identity(), initial_joints, R_target );

	// Solve IK
	std::vector< double > seeds = { 0.0, 0.0, 0.0 };
	SolverResult result = three_solver.IK( R_target, seeds, 0 );

	// Check that the solution is valid
	EXPECT_EQ( result.state, SolverState::Success ) << "IK should succeed for reachable target";

	// Verify the solution by checking the resulting rotation
	Mat4d R_result;
	POE( *three_joint_chain, Mat4d::Identity(), result.joints, R_result );

	EXPECT_TRUE( IsApprox( R_target, R_result ) )
	    << "Initial joints: { " << initial_joints.transpose() << " }" << std::endl
	    << "Expected Mat\n" << R_target << std::endl
	    << "Result joint: { " << result.joints.transpose() << " }" << std::endl
	    << "Result Mat\n" << R_result << std::endl;
}

// ------------------------------------------------------------

TEST_F( WristSolverTest, IK_FallbackToNumeric )
{
	auto three_joint_chain = std::make_shared< JointChain >( 2 );
	three_joint_chain->Add(
		Twist( Vec3d( 0, 0, 1 ), Vec3d( 0, 0, 0 ) ),
		Link( Mat4d::Identity() ),
		Limits( -M_PI, M_PI )
		);
	three_joint_chain->Add(
		Twist( Vec3d( 0, 1, 0 ), Vec3d( 0, 0, 0 ) ),
		Link( Mat4d::Identity() ),
		Limits( -M_PI, M_PI )
		);
	three_joint_chain->Add(
		Twist( Vec3d( 1, 0, 0 ), Vec3d( 0, 0, 0 ) ),
		Link( Mat4d::Identity() ),
		Limits( -M_PI, M_PI )
		);

	// Create a wrist model for the three joints
	WristModel three_wrist_model;
	three_wrist_model.type = WristType::Revolute3;
	three_wrist_model.active_joint_start = 0;
	three_wrist_model.active_joint_count = 3;
	three_wrist_model.center_at_home = Vec3d( 0, 0, 0 );
	three_wrist_model.tcp_in_wrist_at_home = Mat4d::Identity();
	three_wrist_model.tcp_in_wrist_at_home_inv = Mat4d::Identity();
	auto home = std::make_shared< const Mat4d >( three_wrist_model.tcp_in_wrist_at_home );

	WristSolver three_solver( three_joint_chain, home, three_wrist_model );

	// Create a target transform that might be hard to solve analytically
	Mat4d target;

	target << 0, 0, -1, 0,
	    0, 1, 0, 0,
	    1, 0, 0, 0,
	    0, 0, 0, 1;

	// Seed joints
	std::vector< double > seed_joints = { 0.0, 0.0, 0.0 };
	std::span< const double > seed_joints_span( seed_joints );

	// Solve IK
	SolverResult result = three_solver.IK( target, seed_joints_span, 0 );

	ASSERT_TRUE( result.Success() );
	Mat4d R_result;
	POE( *three_joint_chain, Mat4d::Identity(), result.joints, R_result );

	EXPECT_TRUE( IsApprox( target, R_result ) );
}

// ------------------------------------------------------------

} // namespace SOArm100::Kinematics::Test
