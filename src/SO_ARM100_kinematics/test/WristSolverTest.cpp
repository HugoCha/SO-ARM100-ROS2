#include "WristSolver.hpp"

#include "Global.hpp"
#include "JointChain.hpp"
#include "KinematicsUtils.hpp"
#include "RobotModelTestData.hpp"
#include "WristModel.hpp"

#include <gtest/gtest.h>

namespace SOArm100::Kinematics::Test
{

// ------------------------------------------------------------
// ------------------------------------------------------------

class WristSolverTest : public ::testing::Test
{
protected:
void SetUp() override
{
	// Create a simple revolute joint chain for testing
	joint_chain_ = Data::GetRevoluteOnlyRobotJointChain();

	// Create a wrist model for the last 3 joints
	wrist_model_.type = WristType::Revolute3;
	wrist_model_.active_joint_start = joint_chain_.GetActiveJointCount() - 3;
	wrist_model_.active_joint_count = 3;
	wrist_model_.center_at_home = Vec3d( 0, 0, 0 );
	wrist_model_.tcp_in_wrist_at_home = Mat4d::Identity();
	wrist_model_.tcp_in_wrist_at_home_inv = Mat4d::Identity();

	// Initialize the solver
	solver_.Initialize( joint_chain_, wrist_model_, 0.01 );
}

void TearDown() override
{
}

JointChain joint_chain_{0};
WristModel wrist_model_{};
WristSolver solver_{};
};

// ------------------------------------------------------------
// ------------------------------------------------------------

TEST_F( WristSolverTest, ComputeWristCenter )
{
	// Create a target transform
	Mat4d target = Mat4d::Identity();
	target.block< 3, 3 >( 0, 0 ) = Eigen::AngleAxisd( M_PI / 4, Vec3d( 0, 0, 1 ) ).toRotationMatrix();
	target.block< 3, 1 >( 0, 3 ) = Vec3d( 1.0, 0.0, 0.0 );

	// Compute the wrist center
	Mat4d wrist_center;
	solver_.ComputeWristCenter( target, wrist_center );

	// The wrist center should be the target transform multiplied by the inverse of the TCP transform
	// Since the TCP transform is identity, the wrist center should be the same as the target
	EXPECT_TRUE( wrist_center.isApprox( target, 1e-6 ) )
	    << "Wrist center should match target when TCP transform is identity";
}

// ------------------------------------------------------------

TEST_F( WristSolverTest, SolveRevolute1 )
{
	// Create a joint chain with a single revolute joint
	JointChain single_joint_chain( 1 );
	single_joint_chain.Add(
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

	// Initialize a solver for the single joint
	WristSolver single_solver;
	single_solver.Initialize( single_joint_chain, single_wrist_model, 0.01 );

	// Test with a target rotation around the Z-axis
	Mat4d R_target = Mat4d::Identity();
    R_target.block<3,3>(0,0) = Eigen::AngleAxisd( M_PI / 4, Vec3d( 0, 0, 1 ) ).toRotationMatrix();

	// Solve IK
	std::vector< double > seeds = { 0.0 };
	WristSolverResult result = single_solver.IK( R_target, seeds );

	// Check that the solution is valid
	EXPECT_EQ( result.state, WristSolverState::Success ) << "IK should succeed for reachable target";
	EXPECT_NEAR( result.joints[0], M_PI / 4, 1e-6 ) << "Solution should match target angle";

	// Verify the solution by checking the resulting rotation
	Mat3d R_result = Eigen::AngleAxisd( result.joints[0], Vec3d( 0, 0, 1 ) ).toRotationMatrix();
	EXPECT_TRUE( R_result.isApprox( R_target.block<3,3>(0,0), 1e-6 ) ) << "Resulting rotation should match target";
}

// ------------------------------------------------------------

TEST_F( WristSolverTest, SolveRevolute1_Unreachable )
{
	// Create a joint chain with a single revolute joint
	JointChain single_joint_chain( 1 );
	single_joint_chain.Add(
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

	// Initialize a solver for the single joint
	WristSolver single_solver;
	single_solver.Initialize( single_joint_chain, single_wrist_model, 0.01 );

	// Test with a target rotation around the X-axis (unreachable for a Z-axis joint)
    Mat4d R_target = Mat4d::Identity();
    R_target.block<3,3>(0,0) = Eigen::AngleAxisd( M_PI / 4, Vec3d( 1, 0, 0 ) ).toRotationMatrix();

	// Solve IK
	std::vector< double > seeds = { 0.0 };
	WristSolverResult result = single_solver.IK( R_target, seeds );

	// Check that the solution is unreachable
	EXPECT_EQ( result.state, WristSolverState::Unreachable ) << "IK should fail for unreachable target";
}

// ------------------------------------------------------------

TEST_F( WristSolverTest, SolveRevolute2 )
{
	// Create a joint chain with two revolute joints
	JointChain two_joint_chain( 2 );
	two_joint_chain.Add(
		Twist( Vec3d( 0, 0, 1 ), Vec3d( 0, 0, 0 ) ),
		Link( Mat4d::Identity() ),
		Limits( -M_PI, M_PI )
		);
	two_joint_chain.Add(
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

	// Initialize a solver for the two joints
	WristSolver two_solver;
	two_solver.Initialize( two_joint_chain, two_wrist_model, 0.01 );

	// Test with a target rotation
    // Mat4d R_target = Mat4d::Identity();
    // R_target.block<3,3>(0,0) = Eigen::AngleAxisd( M_PI / 3, Vec3d( 0, 0, 1 ) ).toRotationMatrix() *
	//                  Eigen::AngleAxisd( M_PI / 6, Vec3d( 0, 1, 0 ) ).toRotationMatrix();
    Mat4d R_target;
    VecXd initial_joints(2);
    initial_joints << M_PI / 3, M_PI / 6;

    POE( joint_chain_, Mat4d::Identity(), initial_joints, R_target );

	// Solve IK
	std::vector< double > seeds = { 0.0, 0.0 };
	WristSolverResult result = two_solver.IK( R_target, seeds );

	// Check that the solution is valid
	EXPECT_EQ( result.state, WristSolverState::Success ) << "IK should succeed for reachable target";

	// Verify the solution by checking the resulting rotation
	Mat4d R_result;
    VecXd result_joints(2);
    result_joints << result.joints[0], result.joints[1];
    POE( joint_chain_, Mat4d::Identity(), result_joints, R_result );
	
    EXPECT_TRUE( R_result.isApprox( R_target, 1e-6 ) ) 
        << "Initial joints: { " << initial_joints.transpose() << " }\n" 
        << "\nExpected Mat\n" << R_target
        << "Result joint: { " << result.joints.transpose() << " }\n" 
        << "Result Mat\n" << R_result;
}

// ------------------------------------------------------------

TEST_F( WristSolverTest, SolveRevolute2_Unreachable )
{
	// Create a joint chain with two parallel revolute joints
	JointChain two_joint_chain( 2 );
	two_joint_chain.Add(
		Twist( Vec3d( 0, 0, 1 ), Vec3d( 0, 0, 0 ) ),
		Link( Mat4d::Identity() ),
		Limits( -M_PI, M_PI )
		);
	two_joint_chain.Add(
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

	// Initialize a solver for the two joints
	WristSolver two_solver;
	two_solver.Initialize( two_joint_chain, two_wrist_model, 0.01 );

	// Test with a target rotation
    Mat4d R_target = Mat4d::Identity();
    R_target.block<3,3>(0,0) = Eigen::AngleAxisd( M_PI / 4, Vec3d( 0, 0, 1 ) ).toRotationMatrix() *
	                 Eigen::AngleAxisd( M_PI / 6, Vec3d( 0, 1, 0 ) ).toRotationMatrix();

	// Solve IK
	std::vector< double > seeds = { 0.0, 0.0 };
	WristSolverResult result = two_solver.IK( R_target, seeds );

	// Check that the solution is unreachable (because the axes are parallel)
	EXPECT_EQ( result.state, WristSolverState::Unreachable ) << "IK should fail for parallel axes";
}

// ------------------------------------------------------------

TEST_F( WristSolverTest, SolveRevolute3 )
{
	// Create a joint chain with three revolute joints (spherical wrist)
	JointChain three_joint_chain( 3 );
	three_joint_chain.Add(
		Twist( Vec3d( 0, 0, 1 ), Vec3d( 0, 0, 0 ) ),
		Link( Mat4d::Identity() ),
		Limits( -M_PI, M_PI )
		);
	three_joint_chain.Add(
		Twist( Vec3d( 0, 1, 0 ), Vec3d( 0, 0, 0 ) ),
		Link( Mat4d::Identity() ),
		Limits( -M_PI, M_PI )
		);
	three_joint_chain.Add(
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

	// Initialize a solver for the three joints
	WristSolver three_solver;
	three_solver.Initialize( three_joint_chain, three_wrist_model, 0.01 );

	// Test with a target rotation
    Mat4d R_target = Mat4d::Identity();
    R_target.block<3,3>(0,0) = Eigen::AngleAxisd( M_PI / 4, Vec3d( 0, 0, 1 ) ).toRotationMatrix() *
	                 Eigen::AngleAxisd( M_PI / 6, Vec3d( 0, 1, 0 ) ).toRotationMatrix() *
	                 Eigen::AngleAxisd( M_PI / 8, Vec3d( 1, 0, 0 ) ).toRotationMatrix();

	// Solve IK
	std::vector< double > seeds = { 0.0, 0.0, 0.0 };
	WristSolverResult result = three_solver.IK( R_target, seeds );

	// Check that the solution is valid
	EXPECT_EQ( result.state, WristSolverState::Success ) << "IK should succeed for reachable target";

	// Verify the solution by checking the resulting rotation
	Mat3d R_result = Eigen::AngleAxisd( result.joints[0], Vec3d( 0, 0, 1 ) ).toRotationMatrix() *
	                 Eigen::AngleAxisd( result.joints[1], Vec3d( 0, 1, 0 ) ).toRotationMatrix() *
	                 Eigen::AngleAxisd( result.joints[2], Vec3d( 1, 0, 0 ) ).toRotationMatrix();
	EXPECT_TRUE( R_result.isApprox( R_target.block<3,3>(0,0), 1e-6 ) ) << "Resulting rotation should match target";
}

// ------------------------------------------------------------

TEST_F( WristSolverTest, SolveRevolute3_Singularity )
{
	// Create a joint chain with three revolute joints (spherical wrist)
	JointChain three_joint_chain( 3 );
	three_joint_chain.Add(
		Twist( Vec3d( 0, 0, 1 ), Vec3d( 0, 0, 0 ) ),
		Link( Mat4d::Identity() ),
		Limits( -M_PI, M_PI )
		);
	three_joint_chain.Add(
		Twist( Vec3d( 0, 1, 0 ), Vec3d( 0, 0, 0 ) ),
		Link( Mat4d::Identity() ),
		Limits( -M_PI, M_PI )
		);
	three_joint_chain.Add(
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

	// Initialize a solver for the three joints
	WristSolver three_solver;
	three_solver.Initialize( three_joint_chain, three_wrist_model, 0.01 );

	// Test with a target rotation that causes a singularity (gimbal lock)
    Mat4d R_target = Mat4d::Identity();
    R_target.block<3,3>(0,0) = Eigen::AngleAxisd( M_PI / 2, Vec3d( 0, 1, 0 ) ).toRotationMatrix() *
	                 Eigen::AngleAxisd( M_PI / 2, Vec3d( 1, 0, 0 ) ).toRotationMatrix();

	// Solve IK
	std::vector< double > seeds = { 0.0, 0.0, 0.0 };
	WristSolverResult result = three_solver.IK( R_target, seeds );

	// Check that the solution is a singularity
	EXPECT_EQ( result.state, WristSolverState::Singularity ) << "IK should detect singularity for gimbal lock";
}

// ------------------------------------------------------------

TEST_F( WristSolverTest, IK_FallbackToNumeric )
{
	// Create a target transform that might be hard to solve analytically
	Mat4d target = Mat4d::Identity();
	target.block< 3, 3 >( 0, 0 ) = Eigen::AngleAxisd( 1.23, Vec3d( 0.577, 0.577, 0.577 ).normalized() ).toRotationMatrix();

	// Seed joints
	std::vector< double > seed_joints = { 0.0, 0.0, 0.0 };
	std::span< const double > seed_joints_span( seed_joints );

	// Solve IK
	WristSolverResult result = solver_.IK( target, seed_joints_span );

	// Check that the solution is valid (might use numeric solver as fallback)
	EXPECT_TRUE( result.Success() || result.Unreachable() ) << "IK should either succeed or detect unreachable target";

	if ( result.Success() )
	{
		// Verify the solution by checking the resulting transform
		Mat4d result_transform = Mat4d::Identity();
		result_transform.block< 3, 3 >( 0, 0 ) =
			Eigen::AngleAxisd( result.joints[0], Vec3d( 0, 0, 1 ) ).toRotationMatrix() *
			Eigen::AngleAxisd( result.joints[1], Vec3d( 0, 1, 0 ) ).toRotationMatrix() *
			Eigen::AngleAxisd( result.joints[2], Vec3d( 1, 0, 0 ) ).toRotationMatrix();
		Mat3d result_rot = result_transform.block< 3, 3 >( 0, 0 );
		EXPECT_TRUE( result_rot.isApprox( target.block< 3, 3 >( 0, 0 ), 1e-4 ) )
		    << "Resulting rotation should approximately match target (with some tolerance for numeric solver)";
	}
}

// ------------------------------------------------------------

} // namespace SOArm100::Kinematics::Test
