#include "HybridSolver/BaseJointSolver.hpp"

#include "Joint/JointChain.hpp"
#include "RobotModelTestData.hpp"
#include "SolverResult.hpp"

#include <gtest/gtest.h>
#include <cmath>
#include <ostream>

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
	base_joint_model_.reference_direction = Vec3d( 1.0, 0.0, 0.0 );
}

void TearDown() override
{
}

JointChain joint_chain_{ 0 };
BaseJointModel base_joint_model_{};
};

// ------------------------------------------------------------
// ------------------------------------------------------------

TEST_F( BaseJointSolverTest, FK )
{
	VecXd joints( 1 );
	joints[0] = M_PI / 2.0; // 90 degrees

    BaseJointSolver solver( joint_chain_, base_joint_model_ );

	Mat4d fk;
	solver.FK( joints, fk );

	// Expected transformation: rotation of 90 degrees around Z-axis
	Mat4d expected = Mat4d::Identity();
	expected.block< 3, 3 >( 0, 0 ) = Eigen::AngleAxisd( M_PI / 2, Eigen::Vector3d::UnitZ() ).toRotationMatrix();

	EXPECT_TRUE( fk.isApprox( expected, 1e-6 ) ) << "Forward kinematics should match expected rotation";
}

// ------------------------------------------------------------

TEST_F( BaseJointSolverTest, IK_Success )
{
    Vec3d wrist_center_at_home = {1,0,0};

	// Create a target wrist center position
	Mat4d wrist_center = Mat4d::Identity();
	wrist_center.block< 3, 1 >( 0, 3 ) = Vec3d( sqrt( 2 ) / 2.0, sqrt( 2 ) / 2.0, 0.0 );

	// Seed joint angle
	std::vector< double > seed_joints = { 0.0 };
	std::span< const double > seed_joints_span( seed_joints );

	// Solve IK
    BaseJointSolver solver( joint_chain_, base_joint_model_ );
	SolverResult result = solver.IK( wrist_center, seed_joints_span, 0 );

	// Check that the solution is valid
	EXPECT_EQ( result.state, SolverState::Success ) << "IK should succeed for reachable position";
	EXPECT_FALSE( std::isnan( result.joints[0] ) ) << "Solution should not be NaN";

	// Verify the solution by checking FK with the result
	VecXd joints( 1 );
	joints[0] = result.joints[0];
	Mat4d fk;
	solver.FK( joints, fk );

	Vec3d transformed_wrist_center = fk.block< 3, 3 >( 0, 0 ) * wrist_center_at_home;
	Vec3d expected_position = wrist_center.block< 3, 1 >( 0, 3 ); // Should be the same since rotation is around Z-axis

	EXPECT_TRUE( transformed_wrist_center.isApprox( expected_position, 1e-4 ) )
	    << "wrist center= " << expected_position.transpose() << std::endl
	    << "result joints= " << result.joints * 180 / M_PI << std::endl
	    << "result wrist center= " << transformed_wrist_center.transpose() << std::endl;
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
    BaseJointSolver solver( joint_chain_, base_joint_model_ );
	SolverResult result = solver.IK( wrist_center, seed_joints_span, 0 );

	// Check that the solution is a singularity
	EXPECT_EQ( result.state, SolverState::Singularity ) << "IK should detect singularity";
	EXPECT_EQ( result.joints[0], seed_joints[0] ) << "Solution should match seed joint in singularity case";
}

// ------------------------------------------------------------

TEST_F( BaseJointSolverTest, IK_EdgeCases )
{
	// Test with wrist center at origin
	Mat4d wrist_center = Mat4d::Identity();
	wrist_center.block< 3, 1 >( 0, 3 ) = Vec3d( 0.0, 0.0, 0.0 );

	std::vector< double > seed_joints = { 0.0 };
	std::span< const double > seed_joints_span( seed_joints );

    BaseJointSolver solver( joint_chain_, base_joint_model_ );
	SolverResult result = solver.IK( wrist_center, seed_joints_span, 0 );

	// Should be a singularity since the wrist center is on the axis
	EXPECT_EQ( result.state, SolverState::Singularity ) << "Wrist center at origin should be singularity";
}

// ------------------------------------------------------------

}
