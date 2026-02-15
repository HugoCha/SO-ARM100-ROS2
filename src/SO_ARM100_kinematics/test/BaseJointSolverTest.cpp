#include "BaseJointSolver.hpp"

#include "JointChain.hpp"
#include "RobotModelTestData.hpp"

#include <gtest/gtest.h>
#include <cmath>

namespace SOArm100::Kinematics::Test
{

// ------------------------------------------------------------
// ------------------------------------------------------------

class BaseJointSolverTest : public ::testing::Test
{
protected:
void SetUp() override
{
	// Create a simple revolute joint chain for testing
	joint_chain_ = JointChain( Data::GetRevoluteOnlyRobotJointChain() );

	// Create a base joint model
	BaseJointModel base_joint_model;
	base_joint_model.reference_direction = Vec3d( 1.0, 0.0, 0.0 );

	// Initialize the solver
	solver_.Initialize( joint_chain_, base_joint_model );
}

void TearDown() override
{
}

JointChain joint_chain_{ 0 };
BaseJointSolver solver_{};
};

// ------------------------------------------------------------
// ------------------------------------------------------------

TEST_F( BaseJointSolverTest, FK )
{
	VecXd joints( 1 );
	joints[0] = M_PI / 2.0; // 90 degrees

	Mat4d fk;
	solver_.FK( joints, fk );

	// Expected transformation: rotation of 90 degrees around Z-axis
	Mat4d expected = Mat4d::Identity();
	expected.block< 3, 3 >( 0, 0 ) = Eigen::AngleAxisd( M_PI / 2, Eigen::Vector3d::UnitZ() ).toRotationMatrix();

	EXPECT_TRUE( fk.isApprox( expected, 1e-6 ) ) << "Forward kinematics should match expected rotation";
}

// ------------------------------------------------------------

TEST_F( BaseJointSolverTest, IK_Success )
{
	// Create a target wrist center position
	Mat4d wrist_center = Mat4d::Identity();
	wrist_center.block< 3, 1 >( 0, 3 ) = Vec3d( 0.5, 0.5, 0.0 );

	// Seed joint angle
	std::vector< double > seed_joints = { 0.0 };
	std::span< const double > seed_joints_span( seed_joints );

	// Solve IK
	BaseJointSolverResult result = solver_.IK( wrist_center, seed_joints_span );

	// Check that the solution is valid
	EXPECT_EQ( result.state, BaseJointSolverState::Success ) << "IK should succeed for reachable position";
	EXPECT_FALSE( std::isnan( result.base_joint[0] ) ) << "Solution should not be NaN";

	// Verify the solution by checking FK with the result
	VecXd joints( 1 );
	joints[0] = result.base_joint[0];
	Mat4d fk;
	solver_.FK( joints, fk );

	// The wrist center should be at the expected position after applying the transform
	Vec3d transformed_wrist_center = fk.block< 3, 3 >( 0, 0 ) * wrist_center.block< 3, 1 >( 0, 3 ) + fk.block< 3, 1 >( 0, 3 );
	Vec3d expected_position = wrist_center.block< 3, 1 >( 0, 3 ); // Should be the same since rotation is around Z-axis

	EXPECT_TRUE( transformed_wrist_center.isApprox( expected_position, 1e-4 ) )
	    << "Forward kinematics with IK solution should match target position";
}

// ------------------------------------------------------------

TEST_F( BaseJointSolverTest, IK_Singularity )
{
	// Create a target wrist center position that is on the axis of rotation
	Mat4d wrist_center = Mat4d::Identity();
	wrist_center.block< 3, 1 >( 0, 3 ) = Vec3d( 0.0, 0.0, 1.0 ); // On the Z-axis

	// Seed joint angle
	std::vector< double > seed_joints = { 0.0 };
	std::span< const double > seed_joints_span( seed_joints );

	// Solve IK
	BaseJointSolverResult result = solver_.IK( wrist_center, seed_joints_span );

	// Check that the solution is a singularity
	EXPECT_EQ( result.state, BaseJointSolverState::Singularity ) << "IK should detect singularity";
	EXPECT_EQ( result.base_joint[0], seed_joints[0] ) << "Solution should match seed joint in singularity case";
}

// ------------------------------------------------------------

TEST_F( BaseJointSolverTest, IK_Unreachable )
{
	// Create a target wrist center position that is unreachable
	Mat4d wrist_center = Mat4d::Identity();
	wrist_center.block< 3, 1 >( 0, 3 ) = Vec3d( 0.0, 0.0, -1.0 ); // Negative Z (assuming positive Z is the axis)

	// But set reference direction to make it unreachable
	BaseJointModel model;
	model.reference_direction = Vec3d( 0.0, 0.0, 1.0 ); // Same as rotation axis
	BaseJointSolver custom_solver;
	custom_solver.Initialize( joint_chain_, model );

	// Seed joint angle
	std::vector< double > seed_joints = { 0.0 };
	std::span< const double > seed_joints_span( seed_joints );

	// Solve IK
	BaseJointSolverResult result = custom_solver.IK( wrist_center, seed_joints_span );

	// Check that the solution is unreachable
	EXPECT_EQ( result.state, BaseJointSolverState::Unreachable ) << "IK should detect unreachable position";
	EXPECT_TRUE( std::isnan( result.base_joint[0] ) ) << "Solution should be NaN for unreachable position";
}

// ------------------------------------------------------------

TEST_F( BaseJointSolverTest, IK_EdgeCases )
{
	// Test with wrist center at origin
	Mat4d wrist_center = Mat4d::Identity();
	wrist_center.block< 3, 1 >( 0, 3 ) = Vec3d( 0.0, 0.0, 0.0 );

	std::vector< double > seed_joints = { 0.0 };
	std::span< const double > seed_joints_span( seed_joints );

	BaseJointSolverResult result = solver_.IK( wrist_center, seed_joints_span );

	// Should be a singularity since the wrist center is on the axis
	EXPECT_EQ( result.state, BaseJointSolverState::Singularity ) << "Wrist center at origin should be singularity";
}

// ------------------------------------------------------------

}
