#include "HybridSolver/BaseNumericWristSolver.hpp"

#include "Global.hpp"
#include "HybridSolver/BaseJointAnalyzer.hpp"
#include "HybridSolver/BaseJointModel.hpp"
#include "HybridSolver/WristCenterJointsModel.hpp"
#include "HybridSolver/WristCenterJointsModel.hpp"
#include "HybridSolver/WristModel.hpp"
#include "Joint/JointChain.hpp"
#include "Joint/Limits.hpp"
#include "RobotModelTestData.hpp"
#include "SolverResult.hpp"
#include "Utils/KinematicsUtils.hpp"
#include "Utils/Converter.hpp"

#include <gtest/gtest.h>
#include <memory>
#include <ostream>

namespace SOArm100::Kinematics::Test
{

// ------------------------------------------------------------
// ------------------------------------------------------------

class BaseNumericWristSolverTest : public ::testing::Test
{
protected:
void SetUp() override
{
	// Create a test joint chain
	joint_chain_ = std::make_shared< JointChain >( Data::Create5DofRobotJointChain() );

	// Create a home configuration
	auto home_configuration = ToTransformMatrix( Vec3d( 0, 0, 1.5 ) );
	home_configuration_ = std::make_shared< Mat4d >( home_configuration );

	// Create a numeric joints model
	wrist_center_model_.start_index = 1;           // Two numeric joints
	wrist_center_model_.count = 2;           // Two numeric joints
	wrist_center_model_.home_configuration = *home_configuration_;

	// Create a wrist model for the last 3 joints
	wrist_model_.type = WristType::Revolute3;
	wrist_model_.active_joint_start = 3;      // After the base and numeric joints
	wrist_model_.active_joint_count = 3;
	wrist_model_.center_at_home = Vec3d( 0, 0, 1.5 );
	wrist_model_.tcp_in_wrist_at_home = Mat4d::Identity();
	wrist_model_.tcp_in_wrist_at_home_inv = Mat4d::Identity();

	// Create a base joint model
	base_model_.reference_direction = Vec3d( 1.0, 0.0, 0.0 );
	// auto base_model = BaseJointAnalyzer::Analyze( *joint_chain_, wrist_model_ );
	// base_model_ = *base_model;

	// Initialize the solver
	solver_ = std::make_unique< BaseNumericWristSolver >(
		joint_chain_,
		home_configuration_,
		base_model_,
		wrist_center_model_,
		wrist_model_
		);
}

void TearDown() override
{
}

const VecXd RandomValidJoints( double margin_percent = 0.05 )
{
	VecXd random( joint_chain_->GetActiveJointCount() );
	for ( size_t i = 0; i < joint_chain_->GetActiveJointCount(); i++ )
		joint_chain_->GetActiveJointLimits( i ).Random( rng_, & random.data()[i], margin_percent );
	return random;
}

const VecXd RandomValidJointsNear( const VecXd& joints, double distance = 0.1, double margin_percent = 0.05 )
{
	VecXd random( joint_chain_->GetActiveJointCount() );
	for ( size_t i = 0; i < joint_chain_->GetActiveJointCount(); i++ )
		joint_chain_->GetActiveJointLimits( i ).RandomNear( rng_, joints[i], & random.data()[i], distance, margin_percent );
	return random;
}

std::shared_ptr< JointChain > joint_chain_;
std::shared_ptr< Mat4d > home_configuration_;
BaseJointModel base_model_;
WristCenterJointsModel wrist_center_model_;
WristModel wrist_model_;
std::unique_ptr< BaseNumericWristSolver > solver_;
random_numbers::RandomNumberGenerator rng_;
};

// ------------------------------------------------------------
// ------------------------------------------------------------

TEST_F( BaseNumericWristSolverTest, IK_Success )
{
	// Create a target pose
	Mat4d target_pose;
	VecXd joints( 6 );
	//joints << 0, M_PI / 4, -M_PI / 4, 0, 0, 0;
	joints << M_PI / 3, M_PI / 4, -M_PI / 4, 0.3, 0.1, -0.5;

    //joints = RandomValidJoints( 0.05 );
	POE( *joint_chain_, *home_configuration_, joints, target_pose );

	// Seed joints
	std::vector< double > seed_joints{ 0.3, 0.4, 0.3, 0.5, 0.5, 0.7 };

	// Solve IK
	SolverResult result = solver_->IK( target_pose, seed_joints, 0.01 );

	Mat4d result_pose;
	POE( *joint_chain_, *home_configuration_, result.joints, result_pose );
	// Check that the solution is valid
	EXPECT_TRUE( result.Success() )
	    << "IK should succeed"
	    << "Result state= " << ( int )result.state << std::endl
	    << "Result joints= " << result.joints.transpose() << std::endl
	    << "Target=\n" << target_pose << std::endl
	    << "Result=\n" << result_pose << std::endl
	    << "Diff=\n" << target_pose - result_pose << std::endl;

	// Check that we have the correct number of joint values
	EXPECT_EQ( result.joints.size(), joint_chain_->GetActiveJointCount() )
	    << "Solution should contain values for all joints";
}

// ------------------------------------------------------------

TEST_F( BaseNumericWristSolverTest, IK_WithDifferentSeedJoints )
{
	// Create a target pose
	Mat4d target_pose = Mat4d::Identity();
	target_pose.block< 3, 3 >( 0, 0 ) = Eigen::AngleAxisd( M_PI / 4, Vec3d( 0, 0, 1 ) ).toRotationMatrix();
	target_pose.block< 3, 1 >( 0, 3 ) = Vec3d( 0.5, 0.5, 1.0 );

	// Test with different seed joints
	std::vector< std::vector< double >> seed_joints_list = {
		{ 0, 0, 0, 0, 0, 0 },
		{ M_PI / 4, M_PI / 4, M_PI / 4, M_PI / 4, M_PI / 4, M_PI / 4 },
		{ -M_PI / 4, -M_PI / 4, -M_PI / 4, -M_PI / 4, -M_PI / 4, -M_PI / 4 },
	};

	for ( const auto& seed_joints : seed_joints_list )
	{
		// Solve IK
		SolverResult result = solver_->IK( target_pose, seed_joints, 0.01 );

		// Check that the solution is valid
		EXPECT_TRUE( result.Success() )
		    << "IK should succeed target with seed joints: "
		    << seed_joints[0];
	}
}

// ------------------------------------------------------------

TEST_F( BaseNumericWristSolverTest, IK_WithDifferentDiscretization )
{
	// Create a target pose
	Mat4d target_pose = Mat4d::Identity();
	target_pose.block< 3, 3 >( 0, 0 ) = Eigen::AngleAxisd( M_PI / 4, Vec3d( 0, 0, 1 ) ).toRotationMatrix();
	target_pose.block< 3, 1 >( 0, 3 ) = Vec3d( 0.5, 0.5, 1.0 );

	// Seed joints
	std::vector< double > seed_joints{ 0, 0, 0, 0, 0, 0 };

	// Test with different discretization values
	std::vector< double > discretizations = { 0.01, 0.1, 0.5 };

	for ( const auto& discretization : discretizations )
	{
		// Solve IK
		SolverResult result = solver_->IK( target_pose, seed_joints, discretization );

		// Check that the solution is valid
		EXPECT_TRUE( result.Success() )
		    << "IK should either succeed or detect unreachable target with discretization: "
		    << discretization;
	}
}

// ------------------------------------------------------------

TEST_F( BaseNumericWristSolverTest, Robustness_GoodSeed_RepeatedCalls )
{
	int NUM_TEST = 1000;

	VecXd joints;
	Mat4d target;
	VecXd good_seed;
	Mat4d ik_result;
	Vec6d error;
	int ik_sucess = 0;

	for ( int i = 0; i < NUM_TEST; ++i )
	{
		joints = RandomValidJoints();
		POE( *joint_chain_, *home_configuration_, joints, target );
		good_seed = RandomValidJointsNear( joints, 0.1 );

		auto result = solver_->IK( target, ToStdVector( good_seed ), 0.01 );

		Mat4d ik_result;
		POE( *joint_chain_, *home_configuration_, result.joints, ik_result );

		if ( result.Success() )
		{
			PoseError( target, ik_result, error );
			EXPECT_LT( error.norm(), error_tolerance )
				<< "error = " << error.transpose() << std::endl
				<< "error norm = " << error.norm() << std::endl;
			ik_sucess++;
		}
	}

	EXPECT_GE( ik_sucess, 0.999 * NUM_TEST );
}

// ------------------------------------------------------------

TEST_F( BaseNumericWristSolverTest, Robustness_RandomSeed_RepeatedCalls )
{
	int NUM_TEST = 1000;

	VecXd joints;
	Mat4d target;
	VecXd random_seed;
	Mat4d ik_result;
	Vec6d error;
	int ik_sucess = 0;

	for ( int i = 0; i < NUM_TEST; ++i )
	{
		joints = RandomValidJoints();
		POE( *joint_chain_, *home_configuration_, joints, target );
		random_seed = RandomValidJoints();

		auto result = solver_->IK( target, ToStdVector( random_seed ), 0.01 );

		POE( *joint_chain_, *home_configuration_, joints, ik_result );

		if ( result.Success() )
		{
			PoseError( target, ik_result, error );
			EXPECT_LT( error.norm(), error_tolerance )
				<< "error = " << error.transpose() << std::endl
				<< "error norm = " << error.norm() << std::endl;
			ik_sucess++;
		}
	}

	EXPECT_GE( ik_sucess, 0.995 * NUM_TEST );
}

// ------------------------------------------------------------

}
