#include "HybridSolver/BaseWristSolver.hpp"

#include "Global.hpp"
#include "HybridSolver/BaseJointModel.hpp"
#include "HybridSolver/WristModel.hpp"
#include "Joint/JointChain.hpp"
#include "Joint/Twist.hpp"
#include "Joint/Link.hpp"
#include "Joint/Limits.hpp"
#include "SolverResult.hpp"
#include "Utils/KinematicsUtils.hpp"

#include <cmath>
#include <gtest/gtest.h>
#include <memory>
#include <vector>

namespace SOArm100::Kinematics::Test
{

// ------------------------------------------------------------
// Helper function to create a test joint chain
// ------------------------------------------------------------

std::shared_ptr< const JointChain > CreateTestJointChain()
{
	// Create a joint chain with 6 joints: 1 base joint + 2 wrist joints
	auto joint_chain = std::make_shared< JointChain >( 3 );

	// Base joint (revolute around Z-axis)
	joint_chain->Add(
		Twist( Vec3d( 0, 0, 1 ), Vec3d( 0, 0, 0 ) ),
		Link( Mat4d::Identity() ),
		Limits( -M_PI, M_PI )
		);

	// Wrist joints (2 revolute joints)
	joint_chain->Add(
		Twist( Vec3d( 1, 0, 0 ), Vec3d( 1.5, 0, 0 ) ),
		Link( Mat4d::Identity() ),
		Limits( -M_PI, M_PI )
		);
	joint_chain->Add(
		Twist( Vec3d( 0, 1, 0 ), Vec3d( 1.5, 0, 0 ) ),
		Link( Mat4d::Identity() ),
		Limits( -M_PI, M_PI )
		);

	return joint_chain;
}

// ------------------------------------------------------------
// ------------------------------------------------------------

class BaseWristSolverTest : public ::testing::Test
{
protected:
void SetUp() override
{
	// Create a test joint chain
	joint_chain_ = CreateTestJointChain();

	// Create a base joint model
	base_model_.reference_direction = Vec3d( 1.0, 0.0, 0.0 );

	// Create a wrist model for the last 2 joints
	wrist_model_.type = WristType::Revolute2;
	wrist_model_.active_joint_start = joint_chain_->GetActiveJointCount() - 2;
	wrist_model_.active_joint_count = 2;
	wrist_model_.center_at_home = Vec3d( 1.5, 0, 0 );
	wrist_model_.tcp_in_wrist_at_home = Mat4d::Identity();
	wrist_model_.tcp_in_wrist_at_home_inv = Mat4d::Identity();

	Mat4d home = Mat4d::Identity();
	home.block< 3, 1 >( 0, 3 ) = wrist_model_.center_at_home;
	home_ = std::make_shared< const Mat4d >( home );

	// Initialize the solver
	solver_ = std::make_unique< BaseWristSolver >( joint_chain_, home_, base_model_, wrist_model_ );
}

void TearDown() override
{
}

std::shared_ptr< const JointChain > joint_chain_;
std::shared_ptr< const Mat4d > home_;
BaseJointModel base_model_;
WristModel wrist_model_;
std::unique_ptr< BaseWristSolver > solver_;
};

// ------------------------------------------------------------
// ------------------------------------------------------------

TEST_F( BaseWristSolverTest, IK_Success )
{
	// Create a target pose
	Mat4d target_pose = Mat4d::Identity();
	target_pose.block< 3, 3 >( 0, 0 ) = Eigen::AngleAxisd( M_PI / 4, Vec3d( 0, 0, 1 ) ).toRotationMatrix();
	target_pose.block< 3, 1 >( 0, 3 ) = Vec3d( 0.75 * sqrt( 2 ), 0.75 * sqrt( 2 ), 0 );

	// Seed joints
	std::vector< double > seed_joints{ 0, 0, 0 };

	// Solve IK
	SolverResult result = solver_->IK( target_pose, seed_joints, 0.01 );

	// Check that the solution is valid
	EXPECT_TRUE( result.Success() )
	    << "IK should either succeed or detect unreachable target";
}

// ------------------------------------------------------------

TEST_F( BaseWristSolverTest, IK_UnreachableBase )
{
	// Create a target pose that's unreachable for the base joint
	Mat4d target_pose = Mat4d::Identity();
	target_pose.block< 3, 1 >( 0, 3 ) = Vec3d( 100.0, 0.0, 0.0 );  // Very far away

	// Seed joints
	std::vector< double > seed_joints{ 0, 0, 0 };

	// Solve IK
	SolverResult result = solver_->IK( target_pose, seed_joints, 0.01 );

	// Check that the solution is unreachable
	EXPECT_TRUE( result.Unreachable() ) << "IK should fail for unreachable target";
}

// ------------------------------------------------------------

TEST_F( BaseWristSolverTest, IK_WithDifferentSeedJoints )
{
	// Create a target pose
	Mat4d target_pose = Mat4d::Identity();
	target_pose.block< 3, 3 >( 0, 0 ) = Eigen::AngleAxisd( M_PI / 4, Vec3d( 0, 0, 1 ) ).toRotationMatrix();
	target_pose.block< 3, 1 >( 0, 3 ) = Vec3d( 0.75 * sqrt( 2 ), 0.75 * sqrt( 2 ), 0 );

	// Test with different seed joints
	std::vector< std::vector< double >> seed_joints_list = {
		{ 0, 0, 0 },
		{ M_PI / 4, M_PI / 4, M_PI / 4 },
		{ -M_PI / 4, -M_PI / 4, -M_PI / 4 }
	};

	for ( const auto& seed_joints : seed_joints_list )
	{
		// Solve IK
		SolverResult result = solver_->IK( target_pose, seed_joints, 0.01 );

		// Check that the solution is valid
		EXPECT_TRUE( result.Success() )
		    << "IK should either succeed or detect unreachable target with seed joints: "
		    << seed_joints[0] << std::endl;
	}
}

// ------------------------------------------------------------

TEST_F( BaseWristSolverTest, IK_WithWristOrientation )
{
	// Create a target pose with specific wrist orientation
	Mat4d home;
	home << 1, 0, 0, 1.5,
	    0, 1, 0, 0,
	    0, 0, 1, 0,
	    0, 0, 0, 1;
	Mat4d target_pose;
	VecXd thetas( 3 );
	thetas << M_PI / 6, M_PI / 4, M_PI / 8;
	POE( *joint_chain_, home, thetas, target_pose );

	// Seed joints
	std::vector< double > seed_joints{ 0, 0, 0 };

	// Solve IK
	SolverResult result = solver_->IK( target_pose, seed_joints, 0.01 );

	// Check that the solution is valid
	EXPECT_TRUE( result.Success() )
	    << "IK should either succeed or detect unreachable target with wrist orientation";
}

// ------------------------------------------------------------

}
