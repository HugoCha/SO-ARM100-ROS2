#include "Global.hpp"

#include "DLS/DLSSolver.hpp"

#include "RobotModelTestData.hpp"

#include "DLS/DLSSolverParameters.hpp"
#include "KinematicTestBase.hpp"
#include "Model/KinematicModel.hpp"
#include "Solver/IKProblem.hpp"
#include "Solver/IKRunContext.hpp"
#include "Solver/IKSolverState.hpp"
#include "Utils/Converter.hpp"
#include "Utils/KinematicsUtils.hpp"
#include "Utils/StringConverter.hpp"

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

class DLSSolverTest : public KinematicTestBase
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
	Solver::DefaultDLSSolverParameters parameters;

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

std::unique_ptr< Solver::DLSSolver > CreateSolver( Model::KinematicModelConstPtr model )
{
	Solver::DefaultDLSSolverParameters parameters;

	parameters.rotation_weight        = 1;
	parameters.translation_weight     = 9;

	parameters.max_iterations         = 100;
	parameters.max_stalle_iterations  = 8;

	parameters.min_step               = 0.05;
	parameters.max_step               = 1.0;
	parameters.line_search_factor     = 0.5;

	parameters.min_damping            = 0.01;
	parameters.max_damping            = 0.2;
	parameters.max_dq                 = 0.5;
	parameters.min_sv_tolerance       = 0.001;

	return std::make_unique< Solver::DLSSolver >( model, parameters );
}

random_numbers::RandomNumberGenerator rng_;
std::unique_ptr< Solver::DLSSolver > solver_;
static constexpr double DEFAULT_TOLERANCE = error_tolerance;
};

// ------------------------------------------------------------
// Construction and Initialization Tests
// ------------------------------------------------------------

TEST_F( DLSSolverTest, ConstructorWithValidParameters )
{
	Solver::DefaultDLSSolverParameters params;
	params.max_iterations = 200;
	params.min_damping = 0.001;
	params.max_damping = 1.0;

	EXPECT_NO_THROW( Solver::DLSSolver solver( model_, params ) );
}

// ------------------------------------------------------------

TEST_F( DLSSolverTest, ConstructorWithInvalidParameters )
{
	Solver::DefaultDLSSolverParameters params;
	params.max_iterations = -1;  // Invalid

	EXPECT_THROW( Solver::DLSSolver solver( model_, params ), std::invalid_argument );
}

// ------------------------------------------------------------

TEST_F( DLSSolverTest, ConstructorInvalidDampingRange )
{
	Solver::DefaultDLSSolverParameters params;
	params.min_damping = 1.0;
	params.max_damping = 0.001;  // min > max: invalid

	EXPECT_THROW( Solver::DLSSolver solver( model_, params ), std::invalid_argument );
}

// ------------------------------------------------------------
// InverseKinematic - Success Cases
// ------------------------------------------------------------

TEST_F( DLSSolverTest, SolveIK_HomePosition )
{
	// Target at home position
	VecXd seed_joints{ 3 };
	seed_joints << 0.0, 0.0, 0.0;

	auto problem = CreateProblem( model_, seed_joints, seed_joints );
	auto result = solver_->Solve( problem, Solver::IKRunContext() );

	EXPECT_EQ( result.state, Solver::IKSolverState::Converged );
	EXPECT_TRUE( result.Success() );
	EXPECT_LT( PoseError( model_, problem, result ), DEFAULT_TOLERANCE );
	EXPECT_EQ( result.joints.size(), 3 );

	// Verify the solution reaches the target
	auto achieved_pose = ComputeFK( model_, result.joints );
	EXPECT_TRUE( IsApprox( problem.target, achieved_pose ) );
}

// ------------------------------------------------------------

TEST_F( DLSSolverTest, SolveIK_SimpleReachableTarget )
{
	VecXd thetas{ 3 };
	thetas << -0.826677, -1.06708, -0.68162;
	VecXd seed = model_->GetChain()->RandomValidJointsNear( rng_, thetas, 0.2 );

	auto problem = CreateProblem( model_, seed, thetas );
	auto result = solver_->Solve( problem, Solver::IKRunContext() );

	EXPECT_EQ( result.state, Solver::IKSolverState::Converged );
	EXPECT_TRUE( result.Success() );

	// Verify solution
	auto achieved_pose = ComputeFK( model_, result.joints );
	EXPECT_TRUE( IsApprox( 
		achieved_pose, 
		problem.target, 
		problem.tolerance ) )
	    << "Target = " << std::endl << problem.target.matrix() << std::endl
	    << "Result = " << std::endl << achieved_pose.matrix() << std::endl
	    << "Joints = " << std::endl << result.joints.matrix() << std::endl
		<< "error = " << PoseError( model_, problem, result ) << " > " << problem.tolerance;
	EXPECT_LT( result.iterations, 10 );
}

// ------------------------------------------------------------

TEST_F( DLSSolverTest, SolveIK_RotatedConfiguration )
{
	// Create a target by rotating joint 1 by 90 degrees
	VecXd target_joints{ 3 };
	target_joints << M_PI / 2, 0.0, 0.0;

	auto problem = CreateProblem( model_, Vec3d::Zero(), target_joints );
	auto result = solver_->Solve( problem, Solver::IKRunContext() );

	EXPECT_EQ( result.state, Solver::IKSolverState::Converged );

	auto achieved_pose = ComputeFK( model_, result.joints );
	EXPECT_TRUE( IsApprox( problem.target, achieved_pose ) )
	    << "Target = " << std::endl << problem.target.matrix() << std::endl
	    << "Result = " << std::endl << achieved_pose.matrix() << std::endl
	    << "Joints = " << std::endl << result.joints.matrix() << std::endl;
}

// ------------------------------------------------------------

TEST_F( DLSSolverTest, SolveIK_MultipleSolutions )
{
	// For some targets, there may be multiple IK solutions
	// Test that we get A valid solution (not necessarily the same as seed)
	VecXd target_joints{ 3 };
	target_joints << 0.5, 0.3, -0.2;

	auto problem = CreateProblem( model_, Vec3d::Zero(), target_joints );
	auto result = solver_->Solve( problem, Solver::IKRunContext() );

	EXPECT_TRUE( result.Success() );

	// Verify it reaches the target (may be different joint config)
	auto achieved_pose = ComputeFK( model_, result.joints );
	EXPECT_TRUE( IsApprox( problem.target, achieved_pose ) )
	    << "Target = " << std::endl << problem.target.matrix() << std::endl
	    << "Result = " << std::endl << achieved_pose.matrix() << std::endl
	    << "Joints = " << std::endl << result.joints.matrix() << std::endl;
}

// ------------------------------------------------------------

TEST_F( DLSSolverTest, SolveIK_GoodSeedConvergesFaster )
{
	VecXd target_joints{ 3 };
	target_joints << 0.5, 0.3, -0.2;
	auto target_pose = ComputeFK( model_, target_joints );

	// Test 1: Bad seed
	Vec3d bad_seed = { -1.0, -1.0, -1.0 };

	auto problem = CreateProblem( model_, bad_seed, target_joints );
	auto result_bad = solver_->Solve( problem, Solver::IKRunContext() );

	// Test 2: Good seed (close to target)
	Vec3d good_seed = { 0.6, 0.4, -0.1 };

	problem = CreateProblem( model_, good_seed, target_joints );
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

TEST_F( DLSSolverTest, SolveIK_UnreachableTarget )
{
	// Target far outside workspace (robot arm length is ~1.0m)
	auto target_pose = ToTransformMatrix( Vec3d{ 10, 10, 10 } );

	auto problem = CreateProblem( Vec3d::Zero(), target_pose );
	auto result = solver_->Solve( problem, Solver::IKRunContext() );

	EXPECT_NE( result.state, Solver::IKSolverState::Converged );
	EXPECT_FALSE( result.Success() );
}

// ------------------------------------------------------------

TEST_F( DLSSolverTest, SolveIK_MaxIterationsReached )
{
	// Use very strict parameters to force max iterations
	Solver::DefaultDLSSolverParameters params;
	params.max_iterations = 5;  // Very low
	double error_tolerance = 1e-10;  // Very strict

	Solver::DLSSolver solver( model_, params );

	VecXd target_joints{ 3 };
	target_joints << 0.5, 0.3, -0.2;

	auto problem = CreateProblem( model_, Vec3d::Zero(), target_joints, error_tolerance );
	auto result = solver.Solve( problem, Solver::IKRunContext() );

	std::cout << "target = " << std::endl << problem.target << std::endl;
	std::cout << "seed = " << std::endl << problem.seed.transpose() << std::endl;
	std::cout << "result = " << std::endl << ComputeFK( model_, result.joints ) << std::endl;

	EXPECT_GE( PoseError( model_, problem, result ), error_tolerance );
	EXPECT_EQ( result.state, Solver::IKSolverState::MaxIterations );
	EXPECT_EQ( result.iterations, params.max_iterations );
}

// ------------------------------------------------------------

TEST_F( DLSSolverTest, SolveIK_InvalidSeedSize )
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

TEST_F( DLSSolverTest, RandomRestart_RecoverFromBadSeed )
{
	// Even with a terrible seed, random restart should find a solution
	// if target is reachable
	VecXd target_joints{ 3 };
	target_joints << 0.5, 0.3, -0.2;

	// Very bad seed (near joint limits)
	Vec3d terrible_seed = { 3.0, 3.0, 3.0 };  // Out of bounds

	auto problem = CreateProblem( model_, terrible_seed, target_joints );
	auto result = solver_->Solve( problem, Solver::IKRunContext() );

	// Should recover via random restart
	EXPECT_TRUE( result.Success() )
	    << "Random restart should recover from bad initialization";
}

// ------------------------------------------------------------
// Convergence Criteria Tests
// ------------------------------------------------------------

TEST_F( DLSSolverTest, ConvergenceTolerance_Strict )
{
	Solver::DefaultDLSSolverParameters params;
	double error_tolerance = 1e-6;  // Very strict
	params.max_iterations = 500;

	Solver::DLSSolver solver( model_, params );

	VecXd target_joints( 3 );
	target_joints << 0.3, 0.2, -0.1;

	auto problem = CreateProblem( model_, VecXd::Zero( 3 ), target_joints, error_tolerance );
	auto result = solver_->Solve( problem, Solver::IKRunContext() );

	if ( result.Success() )
	{
		EXPECT_LT( PoseError( model_, problem, result ), error_tolerance );
	}
}

// ------------------------------------------------------------

TEST_F( DLSSolverTest, ConvergenceTolerance_Loose )
{
	Solver::DefaultDLSSolverParameters params;
	double error_tolerance = 1e-2;  // Loose
	params.max_iterations = 50;

	Solver::DLSSolver solver( model_, params );

	VecXd target_joints{ 3 };
	target_joints << 0.3, 0.2, -0.1;

	auto problem = CreateProblem( model_, VecXd::Zero( 3 ), target_joints, error_tolerance );
	auto result = solver_->Solve( problem, Solver::IKRunContext() );

	// Should converge quickly with loose tolerance
	EXPECT_TRUE( result.Success() );
	EXPECT_LT( result.iterations, 30 );
}

// ------------------------------------------------------------
// Edge Cases
// ------------------------------------------------------------

TEST_F( DLSSolverTest, EdgeCase_ZeroLengthMove )
{
	// Target exactly at current position
	VecXd joints{ 3 };
	joints << 0.5, 0.3, -0.2;

	auto problem = CreateProblem( model_, joints, joints );
	auto result = solver_->Solve( problem, Solver::IKRunContext() );

	EXPECT_TRUE( result.Success() );
	EXPECT_EQ( result.iterations, 0 )
	    << "Should converge immediately when already at target";
}

// ------------------------------------------------------------

TEST_F( DLSSolverTest, EdgeCase_JointLimits )
{
	// Target requiring joints at limits
	VecXd at_limit_joints{ 3 };
	at_limit_joints << M_PI * 0.99, M_PI* 0.99, M_PI* 0.99;
	auto target_pose = ComputeFK( model_, at_limit_joints );

	auto problem = CreateProblem( model_, VecXd::Zero( 3 ), at_limit_joints );
	auto result = solver_->Solve( problem, Solver::IKRunContext() );

	// Should either converge or gracefully fail (not crash)
	EXPECT_NO_THROW( auto success = result.Success() );
}

// ------------------------------------------------------------

TEST_F( DLSSolverTest, EdgeCase_NearSingularTarget )
{
	// Create a target at a singular configuration
	// For this robot, singularity occurs when arm is straight
	VecXd singular_joints{ 3 };
	singular_joints << 0.0, 0.0, 0.0;
	VecXd seed( 3 );
	seed << 0.5, 0.5, 0.5;

	auto problem = CreateProblem( model_, seed, singular_joints );
	auto result = solver_->Solve( problem, Solver::IKRunContext() );

	// Should handle gracefully
	EXPECT_NO_THROW( auto success = result.Success() );
}

// ------------------------------------------------------------
// Performance Tests
// ------------------------------------------------------------

TEST_F( DLSSolverTest, Performance_AverageIterations )
{
	// Test average convergence speed over multiple random targets
	const int NUM_TESTS = 10;
	int total_iterations = 0;
	int successes = 0;
	VecXd seed = VecXd::Zero( 3 );

	for ( int i = 0; i < NUM_TESTS; ++i )
	{
		VecXd joints = model_->GetChain()->RandomValidJoints( rng_ );

		auto problem = CreateProblem( model_, seed, joints );
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

TEST_F( DLSSolverTest, Robustness_BadSeedHigherIteration_RepeatedCalls )
{
	// Solver should work correctly even after multiple calls
	Mat4d target_pose;
	Mat4d result_pose;
	VecXd seed = VecXd::Zero( 3 );

	const int NUM_TEST = 10000;
	double avg_iteration = 0;
	int failure = 0;
	for ( int i = 0; i < NUM_TEST; ++i )
	{
		VecXd joints = model_->GetChain()->RandomValidJoints( rng_ );

		auto problem = CreateProblem( model_, seed, joints );
		auto result = solver_->Solve( problem, Solver::IKRunContext() );

		if ( !result.Success() )
		{
			result_pose = ComputeFK( model_, result.joints );
		}

		EXPECT_TRUE( result.Success() )
		    << "Iteration " << i << std::endl
		    << "Joints = " << joints.transpose() << std::endl
		    << "Target = "  << std::endl << target_pose << std::endl
		    << "Result = "  << std::endl << result_pose << std::endl;

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

TEST_F( DLSSolverTest, Robustness_GoodSeed_RepeatedCalls )
{
	// Solver should work correctly even after multiple calls
	Mat4d target_pose;
	Mat4d result_pose;

	int NUM_TEST = 100;
	double average_iterations = 0;

	for ( int i = 0; i < NUM_TEST; ++i )
	{
		VecXd joints = model_->GetChain()->RandomValidJoints( rng_ );
		VecXd good_seed = model_->GetChain()->RandomValidJointsNear( rng_, joints, 0.1 );

		auto problem = CreateProblem( model_, good_seed, joints );
		auto result = solver_->Solve( problem, Solver::IKRunContext() );

		average_iterations += result.iterations / ( double )NUM_TEST;

		if ( !result.Success() )
		{
			result_pose = ComputeFK( model_, result.joints );
		}

		EXPECT_TRUE( result.Success() )
		    << "Iteration " << i << std::endl
		    << "Joints = " << joints.transpose() << std::endl
		    << "Target = " << std::endl << target_pose << std::endl
		    << "Result = " << std::endl << result_pose << std::endl;
	}

	EXPECT_LT( average_iterations, 5 )
	    << "Average iteration = " << average_iterations << std::endl;
}

// ------------------------------------------------------------
// Configuration Tests
// ------------------------------------------------------------

TEST_F( DLSSolverTest, Configuration_GetAndSet )
{
	Solver::DefaultDLSSolverParameters new_params;
	new_params.max_iterations = 150;

	solver_->SetParameters( new_params );

	const auto& retrieved_params = solver_->GetParameters();

	EXPECT_EQ( retrieved_params.max_iterations, 150 );
}

// ------------------------------------------------------------
// Test All Configurations
// ------------------------------------------------------------

TEST_F( DLSSolverTest, Robustness_DifferentRobot_RepeatedCalls )
{
	int NUM_TEST = 100;

	for ( const auto& robot : Data::GetAllRobots() )
	{
		double average_iterations = 0;
		double average_error = 0;
		int k_successes = 0;
		for ( int i = 0; i < NUM_TEST; ++i )
		{
			VecXd joints = robot.second->GetChain()->RandomValidJoints( rng_ );
			VecXd good_seed = robot.second->GetChain()->RandomValidJointsNear( rng_, joints, 0.3 );

			auto problem = CreateProblem( robot.second, good_seed, joints );
			auto solver = CreateSolver( robot.second );
			auto result = solver->Solve( problem, Solver::IKRunContext() );

			average_iterations += result.iterations / ( double )NUM_TEST;

			if ( !result.Success() )
			{
				Mat4d result_pose = ComputeFK( model_, result.joints );

				// std::cout
				// 		<< "Fail to converge for robot " << robot.first << std::endl
				// 		<< "Iterations " << result.iterations << std::endl
				// 		<< "Error " << result.error << std::endl
				// 		<< "Joints = " << joints.transpose() << std::endl
				// 		<< "Target = " << std::endl << problem.target << std::endl
				// 		<< "Result = " << std::endl << result_pose << std::endl;
			}
			else
			{
				k_successes++;
			}
			average_error += result.error / ( double )NUM_TEST;
			average_iterations += result.iterations / ( double )NUM_TEST;
		}

		if ( average_error > error_tolerance )
		{
			std::cout << "Average error fail for robot " << robot.first
			          << " : " << average_error << std::endl;
		}
		if ( average_iterations > 10 )
		{
			std::cout << "Average iteration fail for robot " << robot.first
			          << " : " << average_iterations << std::endl;
		}
		if ( k_successes < 0.99 * NUM_TEST )
		{
			std::cout << "Fail to converge too many times for robot " << robot.first
			          << " : " << NUM_TEST - k_successes << std::endl;
		}
	}
}

// ------------------------------------------------------------
// Gradient Descent
// ------------------------------------------------------------

// TEST_F( DLSSolverTest, SolverParameters_FindOptimal )
// {
// 	auto initial_sp = Solver::DefaultDLSSolverParameters();

//     initial_sp.max_iterations = 200;
//     initial_sp.min_step = 0.1;
//     initial_sp.max_step = 0.9;
//     initial_sp.line_search_factor = 0.8;
//     initial_sp.min_damping = 0.05;
//     initial_sp.max_damping = 0.75;
//     initial_sp.max_dq = 0.75;
// 	initial_sp.min_sv_tolerance = 0.001;
//     initial_sp.max_stalle_iterations = 3;
//     initial_sp.translation_weight  = 9;
//     initial_sp.rotation_weight = 1;

// 	DLSGradientDescent descent( 500 );
// 	DLSGradientDescent::Parameters p;
// 	p.iterations = 1000;
// 	// p.learning_rate = 0.5;
// 	p.noise_percent = 1.0;

// 	//auto sp = initial_sp;
// 	//auto sp = descent.Run( p );
// 	auto sp = descent.RandomizedSearch( initial_sp, p );
// 	double loss = descent.ComputeLoss( sp );
// 	std::cout << "Loss with Randomized Batch 1 = " << loss << std::endl;
// 	std::cout << "Parameters = " << sp << std::endl;

// 	double avg_loss = 0.0;
// 	for ( int i = 0; i < 10; i++ )
// 	{
// 		DLSGradientDescent descent( 1000 );
// 		double loss = descent.ComputeLoss( sp );
// 		std::cout << "Loss with Batch " << i << " = " << loss << std::endl;
// 		avg_loss += loss / ( double )10;
// 	}
// 	std::cout << "Average Loss = " << avg_loss << std::endl;

// 	// p.noise_percent = 0.25;
// 	// sp = descent.RandomizedSearch( sp, p );
// 	// loss = descent.ComputeLoss( sp );
// 	// std::cout << "Loss with Randomized Batch 2 = " << loss << std::endl;
// 	// std::cout << "Parameters = " << sp << std::endl;

// 	// for ( int i = 0; i < 10; i++ )
// 	// {
// 	// 	DLSGradientDescent descent( 50 );
// 	// 	loss = descent.ComputeLoss( sp );
// 	// 	std::cout << "Loss with Batch " << i << " = " << loss << std::endl;
// 	// }

// 	// p.noise_percent = 0.125;
// 	// sp = descent.RandomizedSearch( sp, p );
// 	// loss = descent.ComputeLoss( sp );
// 	// std::cout << "Loss with Randomized Batch 3 = " << loss << std::endl;
// 	// std::cout << "Parameters = " << sp << std::endl;

// 	// for ( int i = 0; i < 10; i++ )
// 	// {
// 	// 	DLSGradientDescent descent( 50 );
// 	// 	loss = descent.ComputeLoss( sp );
// 	// 	std::cout << "Loss with Batch " << i << " = " << loss << std::endl;
// 	// }
// }

// ------------------------------------------------------------

}