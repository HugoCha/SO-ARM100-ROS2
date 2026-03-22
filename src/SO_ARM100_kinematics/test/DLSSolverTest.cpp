#include "Global.hpp"

#include "DLSSolver/DLSSolver.hpp"

#include "Model/KinematicModel.hpp"
#include "RobotModelTestData.hpp"
#include "Solver/IKProblem.hpp"
#include "Solver/IKRunContext.hpp"
#include "Solver/IKSolverState.hpp"
#include "Utils/Converter.hpp"
#include "Utils/KinematicsUtils.hpp"

#include <Eigen/src/Geometry/AngleAxis.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <cmath>
#include <gtest/gtest.h>
#include <memory>
#include <moveit/robot_model/joint_model_group.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <ostream>
#include <random_numbers/random_numbers.h>
#include <rclcpp/rclcpp.hpp>

namespace SOArm100::Kinematics::Test
{

// ------------------------------------------------------------
// Test Fixture
// ------------------------------------------------------------

class DLSKinematicsSolverTest : public ::testing::Test
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

	// Model
	model_ = Data::GetZYZRevoluteRobot();

	// Initialize solver with default parameters
	Solver::DLSSolver::SolverParameters parameters;

	parameters.error_tolerance = DEFAULT_TOLERANCE;

	parameters.rotation_weight        = 1;
	parameters.translation_weight     = 10;

	parameters.max_iterations         = 1000;
	parameters.max_stalle_iterations  = 5;

	parameters.min_step               = 0.05;
	parameters.max_step               = 1.0;
	parameters.line_search_factor     = 0.5;

	parameters.min_damping            = 0.001;
	parameters.max_damping            = 0.8;
	parameters.max_dq                 = 1.25;
	parameters.min_sv_tolerance       = 0.001;
	solver_ = std::make_unique< Solver::DLSSolver >( model_, parameters );
}

void TearDown() override
{
	solver_.reset();
}

Mat4d ComputeFK( const VecXd& joints )
{
	Mat4d fk;
	model_->ComputeFK( joints, fk );
	return fk;
}

Solver::IKProblem CreateProblem( const VecXd& seed, const VecXd& joints )
{
	return CreateProblem( seed, ComputeFK( joints ) );
}

Solver::IKProblem CreateProblem( const VecXd& seed, const Mat4d& target  )
{
	return { 
		target, 
		seed, 
		translation_tolerance, 
		rotation_tolerance, 
		100 };
}

protected:
Model::KinematicModelConstPtr model_;
random_numbers::RandomNumberGenerator rng_;
std::unique_ptr< Solver::DLSSolver > solver_;
static constexpr double DEFAULT_TOLERANCE = error_tolerance;
};

// ------------------------------------------------------------
// Construction and Initialization Tests
// ------------------------------------------------------------

TEST_F( DLSKinematicsSolverTest, ConstructorWithValidParameters )
{
	Solver::DLSSolver::SolverParameters params;
	params.max_iterations = 200;
	params.error_tolerance = 1e-5;
	params.min_damping = 0.001;
	params.max_damping = 1.0;

	EXPECT_NO_THROW( Solver::DLSSolver solver( model_, params ) );
}

// ------------------------------------------------------------

TEST_F( DLSKinematicsSolverTest, ConstructorWithInvalidParameters )
{
	Solver::DLSSolver::SolverParameters params;
	params.max_iterations = -1;  // Invalid
	params.error_tolerance = 1e-5;

	EXPECT_THROW( Solver::DLSSolver solver( model_, params ), std::invalid_argument );
}

// ------------------------------------------------------------

TEST_F( DLSKinematicsSolverTest, ConstructorInvalidNegativeTolerance )
{
	Solver::DLSSolver::SolverParameters params;
	params.error_tolerance = -0.01;  // Invalid

	EXPECT_THROW( Solver::DLSSolver solver( model_, params ), std::invalid_argument );
}

// ------------------------------------------------------------

TEST_F( DLSKinematicsSolverTest, ConstructorInvalidDampingRange )
{
	Solver::DLSSolver::SolverParameters params;
	params.min_damping = 1.0;
	params.max_damping = 0.001;  // min > max: invalid

	EXPECT_THROW( Solver::DLSSolver solver( model_, params ), std::invalid_argument );
}

// ------------------------------------------------------------
// InverseKinematic - Success Cases
// ------------------------------------------------------------

TEST_F( DLSKinematicsSolverTest, SolveIK_HomePosition )
{
	// Target at home position
	VecXd seed_joints{ 3 };
	seed_joints << 0.0, 0.0, 0.0;
	auto problem = CreateProblem( seed_joints, seed_joints );

	auto result = solver_->Solve( problem, Solver::IKRunContext() );

	EXPECT_EQ( result.state, Solver::IKSolverState::Converged );
	EXPECT_TRUE( result.Success() );
	EXPECT_LT( result.error, DEFAULT_TOLERANCE );
	EXPECT_EQ( result.joints.size(), 3 );

	// Verify the solution reaches the target
	auto achieved_pose = ComputeFK( result.joints );
	EXPECT_TRUE( IsApprox( problem.target, achieved_pose ) );
}

// ------------------------------------------------------------

TEST_F( DLSKinematicsSolverTest, SolveIK_SimpleReachableTarget )
{
	VecXd thetas{ 3 };
	thetas << -0.826677, -3.06708, -2.68162;
	VecXd seed = model_->GetChain()->RandomValidJointsNear( rng_, thetas, 0.2 );

	auto problem = CreateProblem( seed, thetas );
	auto result = solver_->Solve( problem, Solver::IKRunContext() );

	EXPECT_EQ( result.state, Solver::IKSolverState::Converged );
	EXPECT_TRUE( result.Success() );

	// Verify solution
	auto achieved_pose = ComputeFK( result.joints );
	EXPECT_TRUE( IsApprox( achieved_pose, problem.target ) )
	    << "Target = " << std::endl << problem.target.matrix() << std::endl
	    << "Result = " << std::endl << achieved_pose.matrix() << std::endl
	    << "Joints = " << std::endl << result.joints.matrix() << std::endl;
	EXPECT_LT( result.iterations, 10 );
}

// ------------------------------------------------------------

TEST_F( DLSKinematicsSolverTest, SolveIK_RotatedConfiguration )
{
	// Create a target by rotating joint 1 by 90 degrees
	VecXd target_joints{ 3 };
	target_joints << M_PI / 2, 0.0, 0.0;

	auto problem = CreateProblem( Vec3d::Zero(), target_joints );
	auto result = solver_->Solve( problem, Solver::IKRunContext() );

	EXPECT_EQ( result.state, Solver::IKSolverState::Converged );

	auto achieved_pose = ComputeFK( result.joints );
	EXPECT_TRUE( IsApprox( problem.target, achieved_pose ) )
	    << "Target = " << std::endl << problem.target.matrix() << std::endl
	    << "Result = " << std::endl << achieved_pose.matrix() << std::endl
	    << "Joints = " << std::endl << result.joints.matrix() << std::endl;
}

// ------------------------------------------------------------

TEST_F( DLSKinematicsSolverTest, SolveIK_MultipleSolutions )
{
	// For some targets, there may be multiple IK solutions
	// Test that we get A valid solution (not necessarily the same as seed)
	VecXd target_joints{ 3 };
	target_joints << 0.5, 0.3, -0.2;

	auto problem = CreateProblem( Vec3d::Zero(), target_joints );
	auto result = solver_->Solve( problem, Solver::IKRunContext() );

	EXPECT_TRUE( result.Success() );

	// Verify it reaches the target (may be different joint config)
	auto achieved_pose = ComputeFK( result.joints );
	EXPECT_TRUE( IsApprox( problem.target, achieved_pose ) )
	    << "Target = " << std::endl << problem.target.matrix() << std::endl
	    << "Result = " << std::endl << achieved_pose.matrix() << std::endl
	    << "Joints = " << std::endl << result.joints.matrix() << std::endl;
}

// ------------------------------------------------------------

TEST_F( DLSKinematicsSolverTest, SolveIK_GoodSeedConvergesFaster )
{
	VecXd target_joints{ 3 };
	target_joints << 0.5, 0.3, -0.2;
	auto target_pose = ComputeFK( target_joints );

	// Test 1: Bad seed
	Vec3d bad_seed = { -1.0, -1.0, -1.0 };
	auto problem = CreateProblem( bad_seed, target_joints );
	auto result_bad = solver_->Solve( problem, Solver::IKRunContext() );

	// Test 2: Good seed (close to target)
	Vec3d good_seed = { 0.6, 0.4, -0.1 };
	problem = CreateProblem( good_seed, target_joints );
	auto result_good = solver_->Solve( problem, Solver::IKRunContext() );

	EXPECT_TRUE( result_good.Success() );

	// Good seed should converge in fewer iterations
	if ( result_bad.Success() )
	{
		EXPECT_LE( result_good.iterations, result_bad.iterations );
	}
}

// ------------------------------------------------------------
// InverseKinematic - Failure Cases
// ------------------------------------------------------------

TEST_F( DLSKinematicsSolverTest, SolveIK_UnreachableTarget )
{
	// Target far outside workspace (robot arm length is ~1.0m)
	auto target_pose = ToTransformMatrix( Vec3d{ 10, 10, 10 });

	auto problem = CreateProblem( Vec3d::Zero(), target_pose );
	auto result = solver_->Solve( problem, Solver::IKRunContext() );

	EXPECT_NE( result.state, Solver::IKSolverState::Converged );
	EXPECT_FALSE( result.Success() );
}

// ------------------------------------------------------------

TEST_F( DLSKinematicsSolverTest, SolveIK_MaxIterationsReached )
{
	// Use very strict parameters to force max iterations
	Solver::DLSSolver::SolverParameters params;
	params.max_iterations = 5;  // Very low
	params.error_tolerance = 1e-10;  // Very strict

	Solver::DLSSolver solver( model_, params );

	VecXd target_joints{ 3 };
	target_joints << 0.5, 0.3, -0.2;

	auto problem = CreateProblem( Vec3d::Zero(), target_joints );
	auto result = solver_->Solve( problem, Solver::IKRunContext() );

	EXPECT_EQ( result.state, Solver::IKSolverState::MaxIterations );
	EXPECT_EQ( result.iterations, params.max_iterations );
}

// ------------------------------------------------------------

TEST_F( DLSKinematicsSolverTest, SolveIK_InvalidSeedSize )
{
	Mat4d target = Mat4d::Identity();
	auto problem = CreateProblem( VecXd::Zero( 2 ), target );
	auto result = solver_->Solve( problem, Solver::IKRunContext() );

	EXPECT_EQ( result.state, Solver::IKSolverState::NotRun );
	EXPECT_FALSE( result.Success() );
}

// ------------------------------------------------------------
// Random Restart Tests
// ------------------------------------------------------------

TEST_F( DLSKinematicsSolverTest, RandomRestart_RecoverFromBadSeed )
{
	// Even with a terrible seed, random restart should find a solution
	// if target is reachable
	VecXd target_joints{ 3 };
	target_joints << 0.5, 0.3, -0.2;

	// Very bad seed (near joint limits)
	VecXd terrible_seed = { 3.0, 3.0, 3.0 };  // Out of bounds

	auto problem = CreateProblem( terrible_seed, target_joints );
	auto result = solver_->Solve( problem, Solver::IKRunContext() );

	// Should recover via random restart
	EXPECT_TRUE( result.Success() )
	    << "Random restart should recover from bad initialization";
}

// ------------------------------------------------------------
// Convergence Criteria Tests
// ------------------------------------------------------------

TEST_F( DLSKinematicsSolverTest, ConvergenceTolerance_Strict )
{
	Solver::DLSSolver::SolverParameters params;
	params.error_tolerance = 1e-6;  // Very strict
	params.max_iterations = 200;

	Solver::DLSSolver solver( model_, params );

	VecXd target_joints = { 0.3, 0.2, -0.1 };

	auto problem = CreateProblem( VecXd::Zero( 3 ), target_joints );
	auto result = solver_->Solve( problem, Solver::IKRunContext() );

	if ( result.Success() )
	{
		EXPECT_LT( result.error, 1e-6 );
	}
}

// ------------------------------------------------------------

TEST_F( DLSKinematicsSolverTest, ConvergenceTolerance_Loose )
{
	Solver::DLSSolver::SolverParameters params;
	params.error_tolerance = 1e-2;  // Loose
	params.max_iterations = 50;

	Solver::DLSSolver solver( model_, params );

	VecXd target_joints{ 3 };
	target_joints << 0.3, 0.2, -0.1;

	auto problem = CreateProblem( VecXd::Zero( 3 ), target_joints );
	auto result = solver_->Solve( problem, Solver::IKRunContext() );

	// Should converge quickly with loose tolerance
	EXPECT_TRUE( result.Success() );
	EXPECT_LT( result.iterations, 30 );
}

// ------------------------------------------------------------
// Edge Cases
// ------------------------------------------------------------

TEST_F( DLSKinematicsSolverTest, EdgeCase_ZeroLengthMove )
{
	// Target exactly at current position
	VecXd joints{ 3 };
	joints << 0.5, 0.3, -0.2;

	auto problem = CreateProblem( joints, joints );
	auto result = solver_->Solve( problem, Solver::IKRunContext() );

	EXPECT_TRUE( result.Success() );
	EXPECT_EQ( result.iterations, 0 )
	    << "Should converge immediately when already at target";
}

// ------------------------------------------------------------

TEST_F( DLSKinematicsSolverTest, EdgeCase_JointLimits )
{
	// Target requiring joints at limits
	VecXd at_limit_joints{ 3 };
	at_limit_joints << M_PI * 0.99, M_PI* 0.99, M_PI* 0.99;
	auto target_pose = ComputeFK( at_limit_joints );

	auto problem = CreateProblem( VecXd::Zero( 3 ), at_limit_joints );
	auto result = solver_->Solve( problem, Solver::IKRunContext() );

	// Should either converge or gracefully fail (not crash)
	EXPECT_NO_THROW( auto success = result.Success() );
}

// ------------------------------------------------------------

TEST_F( DLSKinematicsSolverTest, EdgeCase_NearSingularTarget )
{
	// Create a target at a singular configuration
	// For this robot, singularity occurs when arm is straight
	VecXd singular_joints{ 3 };
	singular_joints << 0.0, 0.0, 0.0;
	VecXd seed = { 0.5, 0.5, 0.5 };

	auto problem = CreateProblem( seed, singular_joints );
	auto result = solver_->Solve( problem, Solver::IKRunContext() );

	// Should handle gracefully
	EXPECT_NO_THROW( auto success = result.Success() );
}

// ------------------------------------------------------------
// Performance Tests
// ------------------------------------------------------------

TEST_F( DLSKinematicsSolverTest, Performance_AverageIterations )
{
	// Test average convergence speed over multiple random targets
	const int NUM_TESTS = 10;
	int total_iterations = 0;
	int successes = 0;
	VecXd seed { 0.0, 0.0, 0.0 };
	for ( int i = 0; i < NUM_TESTS; ++i )
	{
		VecXd joints = model_->GetChain()->RandomValidJoints( rng_ );

		auto problem = CreateProblem( seed, joints );
		auto result = solver_->Solve( problem, Solver::IKRunContext() );
	
		if ( result.Success() )
		{
			successes++;
			total_iterations += result.iterations;
		}
	}

	EXPECT_GT( successes, NUM_TESTS / 2 ) << "Should solve most targets";

	if ( successes > 0 )
	{
		double avg_iterations = static_cast< double >( total_iterations ) / successes;
		EXPECT_LT( avg_iterations, 40 ) << "Average iterations should be reasonable";
	}
}

// ------------------------------------------------------------
// Robustness Tests
// ------------------------------------------------------------

TEST_F( DLSKinematicsSolverTest, Robustness_BadSeedHigherIteration_RepeatedCalls )
{
	// Solver should work correctly even after multiple calls
	Mat4d target_pose;
	VecXd seed = { 0.0, 0.0, 0.0 };

	const int NUM_TEST = 10000;
	double avg_iteration = 0;
	int failure = 0;
	for ( int i = 0; i < NUM_TEST; ++i )
	{
		VecXd joints = model_->GetChain()->RandomValidJoints( rng_ );

		auto problem = CreateProblem( seed, joints );
		auto result = solver_->Solve( problem, Solver::IKRunContext() );
	
		EXPECT_TRUE( result.Success() )
		    << "Iteration " << i << std::endl
		    << "Joints = " << joints.transpose() << std::endl
		    << "Target = " << target_pose << std::endl
		    << "Result = " << result << std::endl;

		avg_iteration += result.iterations / ( double )NUM_TEST;
		if ( !result.Success() )
		{
			failure++;
		}
	}

	double failure_percent = failure / ( double )NUM_TEST;
	EXPECT_LT( failure_percent, 0.001 )
	    << "Failure percent= " << failure_percent << std::endl;

	EXPECT_LT( avg_iteration, 20 )
	    << "Average Iteration = " << avg_iteration << std::endl;
}

// ------------------------------------------------------------

TEST_F( DLSKinematicsSolverTest, Robustness_GoodSeed_RepeatedCalls )
{
	// Solver should work correctly even after multiple calls
	Mat4d target_pose;

	int NUM_TEST = 100;
	double average_iterations = 0;

	for ( int i = 0; i < NUM_TEST; ++i )
	{
		VecXd joints = model_->GetChain()->RandomValidJoints( rng_ );
		VecXd good_seed = model_->GetChain()->RandomValidJointsNear( rng_, joints, 0.1 );

		auto problem = CreateProblem( good_seed, joints );
		auto result = solver_->Solve( problem, Solver::IKRunContext() );
	
		average_iterations += result.iterations / ( double )NUM_TEST;

		EXPECT_TRUE( result.Success() )
		    << "Iteration " << i << std::endl
		    << "Joints = " << joints.transpose() << std::endl
		    << "Target = " << target_pose << std::endl
		    << "Result = " << result << std::endl;
	}

	EXPECT_LT( average_iterations, 5 )
	    << "Average iteration = " << average_iterations << std::endl;
}

// ------------------------------------------------------------
// Configuration Tests
// ------------------------------------------------------------

TEST_F( DLSKinematicsSolverTest, Configuration_GetAndSet )
{
	Solver::DLSSolver::SolverParameters new_params;
	new_params.max_iterations = 150;
	new_params.error_tolerance = 1e-5;

	solver_->SetParameters( new_params );

	const auto& retrieved_params = solver_->GetParameters();

	EXPECT_EQ( retrieved_params.max_iterations, 150 );
	EXPECT_DOUBLE_EQ( retrieved_params.error_tolerance, 1e-5 );
}

// ------------------------------------------------------------

}