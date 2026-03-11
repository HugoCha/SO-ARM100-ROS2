#include "Global.hpp"

#include "DLSSolver/NumericSolverResult.hpp"
#include "DLSSolver/NumericSolverState.hpp"
#include "RobotModelTestData.hpp"
#include "Utils/Converter.hpp"
#include "Utils/KinematicsUtils.hpp"

#include <Eigen/src/Geometry/AngleAxis.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <cmath>
#include <gtest/gtest.h>
#include <moveit/robot_model/joint_model_group.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <ostream>
#include <rclcpp/rclcpp.hpp>
#include <span>
#include <vector>

// Leave include here
// Expose private/protected method to the test class
#pragma push_macro("private")
#pragma push_macro("protected")
#define private public
#define protected public
#include "DLSSolver/DLSKinematicsSolver.hpp"
#include "KinematicsSolver.hpp"
#pragma pop_macro("protected")
#pragma pop_macro("private")

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

	// Initialize solver with default parameters
	DLSKinematicsSolver::SolverParameters parameters;

    // Use position-only for 3-DOF robot
	parameters.rotation_weight        = 1;
	parameters.translation_weight     = sqrt( 10 );
	
	parameters.error_tolerance        = 1e-6;   // squaredNorm, ~1mm
	parameters.max_iterations         = 200;
	parameters.gradient_tolerance     = 1e-8;
	parameters.max_stalle_iterations  = 5;
	
	parameters.min_step               = 0.01;
	parameters.max_step               = 1.0;
	parameters.line_search_factor 	  = 0.75;
	
	parameters.min_damping            = 0.001;
	parameters.max_damping            = 0.3;
	parameters.max_dq                 = 0.2;
	parameters.min_sv_tolerance       = 0.001;
	solver_ = std::make_unique< DLSKinematicsSolver >(parameters);

	solver_->Initialize(
		Data::GetRevoluteOnlyRobot(),
		"arm",
		"base_link",
		{ "end_effector" },
		0.01 );
}

void TearDown() override
{
	solver_.reset();
}

// Helper: Create a pose at a given position with identity orientation
const Mat4d CreatePose( double x, double y, double z ) const
{
	return CreatePose( x, y, z, Mat3d::Identity() );
}

const Mat4d CreatePose( double x, double y, double z, Mat3d orientation ) const
{
	Mat4d pose = Mat4d::Identity();

	pose.block< 3, 3 >( 0, 0 ) = orientation;
	pose.block< 3, 1 >( 0, 3 ) = Vec3d { x, y, z };

	return pose;
}

const VecXd RandomValidJoints()
{
	const auto& robot_model = Data::GetRevoluteOnlyRobot();
	const auto* joint_model = robot_model->getJointModelGroup( "arm" );
	VecXd random( joint_model->getActiveJointModels().size() );
	random_numbers::RandomNumberGenerator rng;
	joint_model->getVariableRandomPositions( rng, random.data() );
	return random;
}

const Mat4d ComputeFK( const VecXd& joints ) const
{
	Mat4d pose;
	if ( solver_->ForwardKinematic(  joints, pose ) )
	{
		return pose;
	}
	return Mat4d::Identity();
}

protected:
std::unique_ptr< DLSKinematicsSolver > solver_;
static constexpr double DEFAULT_TOLERANCE = error_tolerance;
};

// ------------------------------------------------------------
// Construction and Initialization Tests
// ------------------------------------------------------------

TEST_F( DLSKinematicsSolverTest, DefaultConstructor )
{
	DLSKinematicsSolver solver;

	// Should not throw
	EXPECT_NO_THROW( auto parameters = solver.GetParameters() );
}

// ------------------------------------------------------------

TEST_F( DLSKinematicsSolverTest, ConstructorWithValidParameters )
{
	DLSKinematicsSolver::SolverParameters params;
	params.max_iterations = 200;
	params.error_tolerance = 1e-5;
	params.min_damping = 0.001;
	params.max_damping = 1.0;

	EXPECT_NO_THROW( DLSKinematicsSolver solver( params ) );
}

// ------------------------------------------------------------

TEST_F( DLSKinematicsSolverTest, ConstructorWithInvalidParameters )
{
	DLSKinematicsSolver::SolverParameters params;
	params.max_iterations = -1;  // Invalid
	params.error_tolerance = 1e-5;

	EXPECT_THROW( DLSKinematicsSolver solver( params ), std::invalid_argument );
}

// ------------------------------------------------------------

TEST_F( DLSKinematicsSolverTest, ConstructorInvalidNegativeTolerance )
{
	DLSKinematicsSolver::SolverParameters params;
	params.error_tolerance = -0.01;  // Invalid

	EXPECT_THROW( DLSKinematicsSolver solver( params ), std::invalid_argument );
}

// ------------------------------------------------------------

TEST_F( DLSKinematicsSolverTest, ConstructorInvalidDampingRange )
{
	DLSKinematicsSolver::SolverParameters params;
	params.min_damping = 1.0;
	params.max_damping = 0.001;  // min > max: invalid

	EXPECT_THROW( DLSKinematicsSolver solver( params ), std::invalid_argument );
}

// ------------------------------------------------------------
// InverseKinematic - Success Cases
// ------------------------------------------------------------

TEST_F( DLSKinematicsSolverTest, SolveIK_HomePosition )
{
	// Target at home position
	VecXd seed_joints{3};
	seed_joints << 0.0, 0.0, 0.0;
	auto target_pose = ComputeFK( seed_joints );

	auto result = solver_->InverseKinematic( target_pose, ToStdVector( seed_joints ) );

	EXPECT_EQ( result.state, NumericSolverState::Converged );
	EXPECT_TRUE( result.Success() );
	EXPECT_LT( result.final_error, DEFAULT_TOLERANCE );
	EXPECT_EQ( result.joints.size(), 3 );

	// Verify the solution reaches the target
	auto achieved_pose = ComputeFK( result.joints );
	EXPECT_TRUE( IsApprox( target_pose, achieved_pose ) );
}

// ------------------------------------------------------------

TEST_F( DLSKinematicsSolverTest, SolveIK_SimpleReachableTarget )
{
	// Target: end effector at (0.7, 0.0, 0.0)
	VecXd thetas{3};
	thetas << M_PI / 4, M_PI / 4, M_PI / 8;
	Mat4d target_pose = ComputeFK( thetas );
	std::vector< double > seed_joints = { 0.0, 0.0, 0.0 };
	//std::vector< double > seed_joints = { thetas[0] + 0.05, thetas[1] + 0.05, thetas[2] + 0.05 };

	auto result = solver_->InverseKinematic( target_pose, seed_joints );

	EXPECT_EQ( result.state, NumericSolverState::Converged );
	EXPECT_TRUE( result.Success() );

	// Verify solution
	auto achieved_pose = ComputeFK( result.joints );
	EXPECT_TRUE( IsApprox( achieved_pose, target_pose ) )
	    << "Target = " << std::endl << target_pose.matrix() << std::endl
	    << "Result = " << std::endl << achieved_pose.matrix() << std::endl
	    << "Joints = " << std::endl << result.joints.matrix() << std::endl;
}

// ------------------------------------------------------------

TEST_F( DLSKinematicsSolverTest, SolveIK_RotatedConfiguration )
{
	// Create a target by rotating joint 1 by 90 degrees
	VecXd target_joints{3};
	target_joints << M_PI / 2, 0.0, 0.0;
	auto target_pose = ComputeFK( target_joints );

	std::vector< double > seed_joints = { 0.0, 0.0, 0.0 };

	auto result = solver_->InverseKinematic( target_pose, seed_joints );

	EXPECT_EQ( result.state, NumericSolverState::Converged );

	auto achieved_pose = ComputeFK( result.joints );
	EXPECT_TRUE( IsApprox( target_pose, achieved_pose ) )
	    << "Target = " << std::endl << target_pose.matrix() << std::endl
	    << "Result = " << std::endl << achieved_pose.matrix() << std::endl
	    << "Joints = " << std::endl << result.joints.matrix() << std::endl;
}

// ------------------------------------------------------------

TEST_F( DLSKinematicsSolverTest, SolveIK_MultipleSolutions )
{
	// For some targets, there may be multiple IK solutions
	// Test that we get A valid solution (not necessarily the same as seed)
	VecXd target_joints{3};
	target_joints << 0.5, 0.3, -0.2;
	auto target_pose = ComputeFK( target_joints );

	std::vector< double > seed_joints = { 0.0, 0.0, 0.0 };

	auto result = solver_->InverseKinematic( target_pose, seed_joints );

	EXPECT_TRUE( result.Success() );

	// Verify it reaches the target (may be different joint config)
	auto achieved_pose = ComputeFK( result.joints );
	EXPECT_TRUE( IsApprox( target_pose, achieved_pose ) )
	    << "Target = " << std::endl << target_pose.matrix() << std::endl
	    << "Result = " << std::endl << achieved_pose.matrix() << std::endl
	    << "Joints = " << std::endl << result.joints.matrix() << std::endl;
}

// ------------------------------------------------------------

TEST_F( DLSKinematicsSolverTest, SolveIK_GoodSeedConvergesFaster )
{
	VecXd target_joints{3};
	target_joints << 0.5, 0.3, -0.2;
	auto target_pose = ComputeFK( target_joints );

	// Test 1: Bad seed
	std::vector< double > bad_seed = { -1.0, -1.0, -1.0 };
	auto result_bad = solver_->InverseKinematic( target_pose, bad_seed );

	// Test 2: Good seed (close to target)
	std::vector< double > good_seed = { 0.6, 0.4, -0.1 };
	auto result_good = solver_->InverseKinematic( target_pose, good_seed );

	EXPECT_TRUE( result_good.Success() );

	// Good seed should converge in fewer iterations
	if ( result_bad.Success() )
	{
		EXPECT_LE( result_good.iterations_used, result_bad.iterations_used );
	}
}

// ------------------------------------------------------------
// InverseKinematic - Failure Cases
// ------------------------------------------------------------

TEST_F( DLSKinematicsSolverTest, SolveIK_UnreachableTarget )
{
	// Target far outside workspace (robot arm length is ~1.0m)
	auto target_pose = CreatePose( 10.0, 10.0, 10.0 );
	std::vector< double > seed_joints = { 0.0, 0.0, 0.0 };

	auto result = solver_->InverseKinematic( target_pose, seed_joints );

	EXPECT_NE( result.state, NumericSolverState::Converged );
	EXPECT_FALSE( result.Success() );
}

// ------------------------------------------------------------

TEST_F( DLSKinematicsSolverTest, SolveIK_MaxIterationsReached )
{
	// Use very strict parameters to force max iterations
	DLSKinematicsSolver::SolverParameters params;
	params.max_iterations = 5;  // Very low
	params.error_tolerance = 1e-10;  // Very strict

	DLSKinematicsSolver solver( params );
	solver.Initialize(
		Data::GetRevoluteOnlyRobot(),
		"arm",
		"base_link",
		{ "end_effector" },
		0.01 );

	auto target_pose = CreatePose( 0.5, 0.5, 0.0 );
	std::vector< double > seed_joints = { 0.0, 0.0, 0.0 };

	auto result = solver.InverseKinematic( target_pose, seed_joints );

	EXPECT_EQ( result.state, NumericSolverState::MaxIterations );
	EXPECT_EQ( result.iterations_used, params.max_iterations );
}

// ------------------------------------------------------------

TEST_F( DLSKinematicsSolverTest, SolveIK_InvalidSeedSize )
{
	auto target_pose = CreatePose( 0.5, 0.0, 0.0 );
	std::vector< double > invalid_seed = { 0.0, 0.0 };  // Wrong size (should be 3)

	auto result = solver_->InverseKinematic( target_pose, invalid_seed );

	EXPECT_EQ( result.state, NumericSolverState::Failed );
	EXPECT_FALSE( result.Success() );
}

// ------------------------------------------------------------
// InverseKinematic - Legacy API Tests
// ------------------------------------------------------------

TEST_F( DLSKinematicsSolverTest, InverseKinematic_Success )
{
	VecXd joints{3};
	joints << 0.1, 0.5, 0.7;
	auto reachable_position = ComputeFK( joints );
	std::vector< double > seed_joints = { 0.0, 0.0, 0.0 };
	VecXd solution_joints(3);

	bool success = solver_->InverseKinematicImpl( reachable_position, seed_joints, solution_joints.data() );

	EXPECT_TRUE( success );
	EXPECT_EQ( solution_joints.size(), 3 );

	// Verify solution
	auto achieved_pose = ComputeFK( solution_joints );
	EXPECT_TRUE( IsApprox( reachable_position, achieved_pose ) )
		<< "Expected Pose=\n" << reachable_position << std::endl
		<< "Result Pose=\n" << achieved_pose << std::endl;
}

// ------------------------------------------------------------

TEST_F( DLSKinematicsSolverTest, InverseKinematic_Failure )
{
	auto target_pose = CreatePose( 100.0, 100.0, 100.0 );  // Unreachable
	std::vector< double > seed_joints = { 0.0, 0.0, 0.0 };
	VecXd solution_joints;

	bool success = solver_->InverseKinematicImpl( target_pose, seed_joints, solution_joints.data() );

	EXPECT_FALSE( success );
}

// ------------------------------------------------------------
// Adaptive Parameter Tests
// ------------------------------------------------------------

TEST_F( DLSKinematicsSolverTest, AdaptiveDamping_NearSingularity )
{
	// Create a configuration near singularity (e.g., fully extended arm)
	VecXd near_singular{3};
	near_singular << 0.0, 0.0, 0.001;
	auto target_pose = ComputeFK( near_singular );
	
	// Perturb slightly to force solver to work near singularity
	target_pose( 0, 3 ) += 0.001;

	auto result = solver_->InverseKinematic( target_pose, ToStdVector( near_singular ) );

	// Should still converge (damping helps near singularities)
	EXPECT_TRUE( result.state == NumericSolverState::BestPossible )
	    << "Adaptive damping should handle near-singular configurations";
}

// ------------------------------------------------------------

TEST_F( DLSKinematicsSolverTest, AdaptiveStep_ConvergenceControl )
{
	// Test that adaptive step helps convergence
	VecXd target_joints{3};
	target_joints << 1.0, 0.5, -0.5;
	auto target_pose = ComputeFK( target_joints );

	std::vector< double > far_seed = { -1.5, -1.0, 1.0 };

	auto result = solver_->InverseKinematic( target_pose, far_seed );

	if ( result.Success() )
	{
		// Should converge in reasonable iterations (adaptive step helps)
		EXPECT_LT( result.iterations_used, 100 )
		    << "Adaptive step should help convergence";
	}
}

// ------------------------------------------------------------
// Random Restart Tests
// ------------------------------------------------------------

TEST_F( DLSKinematicsSolverTest, RandomRestart_RecoverFromBadSeed )
{
	// Even with a terrible seed, random restart should find a solution
	// if target is reachable
	VecXd target_joints{3};
	target_joints << 0.5, 0.3, -0.2;
	auto target_pose = ComputeFK( target_joints );

	// Very bad seed (near joint limits)
	std::vector< double > terrible_seed = { 3.0, 3.0, 3.0 };  // Out of bounds

	auto result = solver_->InverseKinematic( target_pose, terrible_seed );

	// Should recover via random restart
	EXPECT_TRUE( result.Success() )
	    << "Random restart should recover from bad initialization";
}

// ------------------------------------------------------------
// Convergence Criteria Tests
// ------------------------------------------------------------

TEST_F( DLSKinematicsSolverTest, ConvergenceTolerance_Strict )
{
	DLSKinematicsSolver::SolverParameters params;
	params.error_tolerance = 1e-6;  // Very strict
	params.max_iterations = 200;

	DLSKinematicsSolver solver( params );
	solver.Initialize(
		Data::GetRevoluteOnlyRobot(),
		"arm",
		"base_link",
		{ "end_effector" },
		0.01 );

	VecXd target_joints{3};
	target_joints << 0.3, 0.2, -0.1;
	auto target_pose = ComputeFK( target_joints );

	std::vector< double > seed = { 0.0, 0.0, 0.0 };

	auto result = solver.InverseKinematic( target_pose, seed );

	if ( result.Success() )
	{
		EXPECT_LT( result.final_error, 1e-6 );
	}
}

// ------------------------------------------------------------

TEST_F( DLSKinematicsSolverTest, ConvergenceTolerance_Loose )
{
	DLSKinematicsSolver::SolverParameters params;
	params.error_tolerance = 1e-2;  // Loose
	params.max_iterations = 50;

	DLSKinematicsSolver solver( params );
	solver.Initialize(
		Data::GetRevoluteOnlyRobot(),
		"arm",
		"base_link",
		{ "end_effector" },
		0.01 );

	VecXd target_joints{3};
	target_joints << 0.3, 0.2, -0.1;
	auto target_pose = ComputeFK( target_joints );

	std::vector< double > seed = { 0.0, 0.0, 0.0 };

	auto result = solver.InverseKinematic( target_pose, seed );

	// Should converge quickly with loose tolerance
	EXPECT_TRUE( result.Success() );
	EXPECT_LT( result.iterations_used, 30 );
}

// ------------------------------------------------------------
// Edge Cases
// ------------------------------------------------------------

TEST_F( DLSKinematicsSolverTest, EdgeCase_ZeroLengthMove )
{
	// Target exactly at current position
	VecXd joints{3};
	joints << 0.5, 0.3, -0.2;
	auto target_pose = ComputeFK( joints );

	auto result = solver_->InverseKinematic( target_pose, ToStdVector( joints ) );

	EXPECT_TRUE( result.Success() );
	EXPECT_EQ( result.iterations_used, 0 )
	    << "Should converge immediately when already at target";
}

// ------------------------------------------------------------

TEST_F( DLSKinematicsSolverTest, EdgeCase_JointLimits )
{
	// Target requiring joints at limits
	VecXd at_limit_joints{3};
	at_limit_joints << M_PI* 0.99, M_PI* 0.99, M_PI* 0.99;
	auto target_pose = ComputeFK( at_limit_joints );

	std::vector< double > seed = { 0.0, 0.0, 0.0 };

	auto result = solver_->InverseKinematic( target_pose, seed );

	// Should either converge or gracefully fail (not crash)
	EXPECT_NO_THROW( auto success = result.Success() );
}

// ------------------------------------------------------------

TEST_F( DLSKinematicsSolverTest, EdgeCase_NearSingularTarget )
{
	// Create a target at a singular configuration
	// For this robot, singularity occurs when arm is straight
	VecXd singular_joints{3};
	singular_joints << 0.0, 0.0, 0.0;
	auto target_pose = ComputeFK( singular_joints );

	std::vector< double > seed = { 0.5, 0.5, 0.5 };

	auto result = solver_->InverseKinematic( target_pose, seed );

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
	std::vector< double > seed { 0.0, 0.0, 0.0 };
	for ( int i = 0; i < NUM_TESTS; ++i )
	{
		Mat4d target_pose;
		bool fk_success = solver_->ForwardKinematic( RandomValidJoints(), target_pose );

		auto result = solver_->InverseKinematic( target_pose, seed );

		if ( result.Success() )
		{
			successes++;
			total_iterations += result.iterations_used;
		}
	}

	EXPECT_GT( successes, NUM_TESTS / 2 ) << "Should solve most targets";

	if ( successes > 0 )
	{
		double avg_iterations = static_cast< double >( total_iterations ) / successes;
		EXPECT_LT( avg_iterations, 50 ) << "Average iterations should be reasonable";
	}
}

// ------------------------------------------------------------
// Robustness Tests
// ------------------------------------------------------------

TEST_F( DLSKinematicsSolverTest, Robustness_RepeatedCalls )
{
	// Solver should work correctly even after multiple calls
	Mat4d target_pose;
	std::vector< double > seed = { 0.0, 0.0, 0.0 };

	for ( int i = 0; i < 5; ++i )
	{
		while ( !solver_->ForwardKinematic( RandomValidJoints(), target_pose ) );

		auto result = solver_->InverseKinematic( target_pose, seed );
		EXPECT_TRUE( result.Success() )
		    << "Iteration " << i << std::endl
		    << "Target = " << target_pose << std::endl
		    << "Result = " << result << std::endl;
	}
}

// ------------------------------------------------------------

TEST_F( DLSKinematicsSolverTest, Robustness_DifferentTargets )
{
	// Test with various targets
	std::vector< std::array< double, 3 >> targets = {
		{ 0.5, 0.0, 0.0 },
		{ 0.3, 0.3, 0.0 },
		{ 0.4, -0.2, 0.1 },
		{ 0.6, 0.1, -0.1 }
	};

	std::vector< double > seed = { 0.0, 0.0, 0.0 };

	for ( const auto& target_pos : targets )
	{
		auto target_pose = CreatePose( target_pos[0], target_pos[1], target_pos[2] );
		auto result = solver_->InverseKinematic( target_pose, seed );

		// At least should not crash
		EXPECT_NO_THROW( auto success = result.Success() );
	}
}

// ------------------------------------------------------------
// Configuration Tests
// ------------------------------------------------------------

TEST_F( DLSKinematicsSolverTest, Configuration_GetAndSet )
{
	DLSKinematicsSolver::SolverParameters new_params;
	new_params.max_iterations = 150;
	new_params.error_tolerance = 1e-5;

	solver_->SetParameters( new_params );

	const auto& retrieved_params = solver_->GetParameters();

	EXPECT_EQ( retrieved_params.max_iterations, 150 );
	EXPECT_DOUBLE_EQ( retrieved_params.error_tolerance, 1e-5 );
}

}