#include "HybridSolver/NumericJointsSolver.hpp"

#include "Global.hpp"
#include "HybridSolver/NumericJointsModel.hpp"
#include "Joint/JointChain.hpp"
#include "RobotModelTestData.hpp"
#include "SolverResult.hpp"
#include "Utils/KinematicsUtils.hpp"

#include <gtest/gtest.h>
#include <memory>
#include <ostream>
#include <vector>

namespace SOArm100::Kinematics::Test
{

// ------------------------------------------------------------
// ------------------------------------------------------------

class NumericJointsSolverTest : public ::testing::Test
{
protected:
void SetUp() override
{
	// Create a simple revolute joint chain for testing
	joint_chain_ = std::make_shared< const JointChain >( Data::GetRevoluteOnlyRobotJointChain() );

	home_ = std::make_shared< const Mat4d >( Data::GetRevoluteOnlyRobotHome() );

	// Create a numeric joints model for the first 3 joints
	numeric_joint_model_.start_index = 0;
	numeric_joint_model_.count = 3;
	numeric_joint_model_.home_configuration = Data::GetRevoluteOnlyRobotHome();
}

void TearDown() override
{
}

std::shared_ptr< const JointChain > joint_chain_;
std::shared_ptr< const Mat4d > home_;
NumericJointsModel numeric_joint_model_{};
};

// ------------------------------------------------------------
// ------------------------------------------------------------

TEST_F( NumericJointsSolverTest, FK )
{
	// Create joint angles
	VecXd joints( 3 );
	joints << M_PI / 4, M_PI / 6, M_PI / 8;

	NumericJointsSolver solver( joint_chain_, home_, numeric_joint_model_ );
	// Compute forward kinematics
	Mat4d fk;
	bool success = solver.FK( joints, fk );

	// Check that the forward kinematics was successful
	EXPECT_TRUE( success ) << "Forward kinematics should succeed";

	// Verify the result by manually computing the expected transform
	Mat4d expected = Mat4d::Identity();
	for ( int i = 0; i < 3; ++i )
	{
		const auto& twist = joint_chain_->GetActiveJointTwist( i );
		expected = expected * twist.ExponentialMatrix( joints[i] );
	}
	expected *= Data::GetRevoluteOnlyRobotHome();
	EXPECT_TRUE( fk.isApprox( expected, 1e-6 ) ) << "Forward kinematics result should match expected transform";
}

// ------------------------------------------------------------

TEST_F( NumericJointsSolverTest, FK_WithSubChain )
{
	// Create a numeric joints model for a subset of joints
	NumericJointsModel sub_model;
	sub_model.start_index = 1;
	sub_model.count = 2;
	sub_model.home_configuration = Mat4d::Identity();

	// Initialize a solver with the sub-chain
	auto sub_home = std::make_shared< const Mat4d >( sub_model.home_configuration );
	NumericJointsSolver sub_solver( joint_chain_, sub_home, sub_model );

	// Create joint angles for the sub-chain
	VecXd joints( 2 );
	joints << M_PI / 6, M_PI / 8;

	// Create full joint angles vector
	VecXd full_joints( 3 );
	full_joints << 0.0, M_PI / 6, M_PI / 8;

	// Compute forward kinematics
	Mat4d fk;
	bool success = sub_solver.FK( joints, fk );

	// Check that the forward kinematics was successful
	EXPECT_TRUE( success ) << "Forward kinematics should succeed";

	// Verify the result by manually computing the expected transform
	Mat4d expected = Mat4d::Identity();
	for ( int i = 1; i < 3; ++i )    // Start from index 1
	{
		const auto& twist = joint_chain_->GetActiveJointTwist( i );
		expected = expected * twist.ExponentialMatrix( full_joints[i] );
	}

	EXPECT_TRUE( fk.isApprox( expected, 1e-6 ) ) << "Forward kinematics result should match expected transform";
}

// ------------------------------------------------------------

TEST_F( NumericJointsSolverTest, IK )
{
	// Create a target pose
	VecXd joints( 3 );
	joints << M_PI / 4, M_PI / 6, M_PI / 8;
	Mat4d target;
	POE( *joint_chain_, Data::GetRevoluteOnlyRobotHome(), joints, target );

	// Seed joints
	std::vector< double > seed_joints{ 0.0, 0.0, 0.0 };

	// Solve IK
	NumericJointsSolver solver( joint_chain_, home_, numeric_joint_model_ );
	SolverResult result = solver.IK( target, seed_joints, 0 );

	// Check that the solution is valid
	EXPECT_TRUE( result.Success() ) << "IK should succeed for reachable target";

	// Verify the solution by checking the resulting transform
	Mat4d fk;
	solver.FK( result.joints, fk );

	EXPECT_TRUE( IsApprox( target, fk ) )
	    << "target=\n" << target << std::endl
	    << "fk=\n" << fk << std::endl
	    << "Result joints=" << result.joints.transpose() * 180 / M_PI << std::endl;
}

// ------------------------------------------------------------

TEST_F( NumericJointsSolverTest, IK_WithSubChain )
{
	// Create a numeric joints model for a subset of joints
	NumericJointsModel sub_model;
	sub_model.start_index = 0;
	sub_model.count = 2;
	sub_model.home_configuration = Data::GetRevoluteOnlyRobotHome();

	// Initialize a solver with the sub-chain
	auto sub_home = std::make_shared< const Mat4d >( sub_model.home_configuration );
	NumericJointsSolver sub_solver( joint_chain_, sub_home, sub_model );

	// Create a target pose for the sub-chain
	VecXd joints( 3 );
	joints << M_PI / 4, M_PI / 6;
	Mat4d target;
	POE( *joint_chain_, Data::GetRevoluteOnlyRobotHome(), joints, target );

	// Seed joints
	std::vector< double > seed_joints{ 0.0, 0.0, 0.0 };

	// Solve IK
	SolverResult result = sub_solver.IK( target, seed_joints, 0 );

	// Check that the solution is valid
	EXPECT_TRUE( result.Success() ) << "IK should succeed for reachable target";

	// Verify the solution by checking the resulting transform
	Mat4d fk;
	sub_solver.FK( result.joints, fk );

	EXPECT_TRUE( IsApprox( target, fk ) )
	    << "target=\n" << target << std::endl
	    << "fk=\n" << fk << std::endl
	    << "Result joints=" << result.joints.transpose() * 180 / M_PI << std::endl;
}

// ------------------------------------------------------------

TEST_F( NumericJointsSolverTest, IK_UnreachableTarget )
{
	// Create an unreachable target pose (too far away)
	Mat4d target = Mat4d::Identity();
	target.block< 3, 1 >( 0, 3 ) = Vec3d( 100.0, 0.0, 0.0 );  // Very far away

	// Seed joints
	std::vector< double > seed_joints{ 0.0, 0.0, 0.0 };

	// Solve IK
	NumericJointsSolver solver( joint_chain_, home_, numeric_joint_model_ );
	SolverResult result = solver.IK( target, seed_joints, 0 );

	// Check that the solution is not successful
	EXPECT_FALSE( result.Success() ) << "IK should fail for unreachable target";
}

// ------------------------------------------------------------

TEST_F( NumericJointsSolverTest, IK_WithDifferentSeedJoints )
{
	// Create a target pose
	VecXd joints( 3 );
	joints << M_PI / 4, M_PI / 6, M_PI / 8;
	Mat4d target;
	POE( *joint_chain_, Data::GetRevoluteOnlyRobotHome(), joints, target );

	// Test with different seed joints
	std::vector< std::vector< double >> seed_joints_list = {
		{ 0, 0, 0 },
		{ M_PI / 4, M_PI / 4, M_PI / 4 },
		{ -M_PI / 4, -M_PI / 4, -M_PI / 4 }
	};

	NumericJointsSolver solver( joint_chain_, home_, numeric_joint_model_ );

	for ( const auto& seed_joints : seed_joints_list )
	{
		// Solve IK
		SolverResult result = solver.IK( target, seed_joints, 0 );

		// Check that the solution is valid
		EXPECT_TRUE( result.Success() ) << "IK should succeed for reachable target with seed joints: "
		                                << seed_joints[0] << std::endl;

		// Verify the solution by checking the resulting transform
		Mat4d fk;
		solver.FK( result.joints, fk );

		EXPECT_TRUE( fk.isApprox( target, 1e-3 ) )
		    << "target=\n" << target << std::endl
		    << "fk=\n" << fk << std::endl
		    << "Result joints=" << result.joints.transpose() * 180 / M_PI << std::endl;
	}
}

// ------------------------------------------------------------

} // namespace SOArm100::Kinematics::Test
