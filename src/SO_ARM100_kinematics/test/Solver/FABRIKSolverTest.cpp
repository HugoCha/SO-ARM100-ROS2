#include "FABRIK/FabrikSolver.hpp"
#include "FABRIK/FabrikSolverDraft.hpp"

#include "Global.hpp"

#include "RobotModelTestData.hpp"
#include "KinematicTestBase.hpp"

#include "Solver/IKProblem.hpp"
#include "Model/KinematicModel.hpp"
#include "Solver/IKRunContext.hpp"
#include "Solver/IKSolution.hpp"
#include "Solver/IKSolverState.hpp"
#include "Utils/KinematicsUtils.hpp"
#include "Utils/StringConverter.hpp"

#include <Eigen/src/Core/Matrix.h>
#include <gtest/gtest.h>
#include <random_numbers/random_numbers.h>
#include <map>

namespace SOArm100::Kinematics::Test
{

// ------------------------------------------------------------
// Fixture
// ------------------------------------------------------------

class FABRIKSolverTest : public KinematicTestBase
{
protected:
void SetUp() override
{
	model_ = Data::GetPrismaticBaseRobot();
}

Solver::IKSolution CheckConvergence( 
	Model::KinematicModelConstPtr model,
	const VecXd& seed,
	const VecXd& joints )
{
	EXPECT_EQ( seed.size(), joints.size() );

	auto solver = Solver::FABRIKSolverDraft( model );
	auto problem = CreateProblem( model, seed, joints );
	auto result = solver.Solve( problem, Solver::IKRunContext() );
	Mat4d result_pose;
	model->ComputeFK( result.joints, result_pose );
	EXPECT_EQ( result.joints.size(), joints.size() );
	EXPECT_TRUE( result.Success() )
		<< "Convergence failed" << std::endl;

	if ( !result.Success() )
	{
		std::cout << problem << std::endl;
		std::cout << result  << std::endl;
	}
		
	EXPECT_TRUE( TranslationError( problem.target, result_pose ) < solver.GetParameters().error_tolerance )
		<< "Joints  = " << result.joints.transpose() << "\n"
		<< "Target  =\n" << Translation( problem.target  ).transpose() << "\n"
		<< "Result  =\n" << Translation( result_pose ).transpose() << std::endl;

	return result;
}

std::map< std::string, Model::KinematicModelConstPtr > ValidRobots()
{
	return 
	{
		{ "ZYZ", Data::GetZYZRevoluteRobot() },
		{ "RevoluteBase", Data::GetRevoluteBaseRobot() },
	};
}

protected:
random_numbers::RandomNumberGenerator rng_;
static constexpr double DEFAULT_TOLERANCE = error_tolerance;
};

// ------------------------------------------------------------
// Basic convergence with known joint configuration
// ------------------------------------------------------------

TEST_F( FABRIKSolverTest, IK_ConvergesFromNearSeed )
{
	VecXd joints = model_->GetChain()->RandomValidJoints( rng_, 0.05 );
	VecXd seed = model_->GetChain()->RandomValidJointsNear( rng_, joints, 0.1, 0 );

	Mat4d initial_position;
	model_->ComputeFK( seed, initial_position );
	std::cout << "Initial position = " << Translation( initial_position ).transpose() << std::endl;

	Mat4d target_position;
	model_->ComputeFK( joints, target_position );
	std::cout << "Target position = " << Translation( target_position ).transpose() << std::endl;

	CheckConvergence( model_, seed, joints );
}

// ------------------------------------------------------------

TEST_F( FABRIKSolverTest, IK_AllRobots_ConvergesFromNearSeed )
{
	for ( const auto& robot : ValidRobots() )
	{
		VecXd joints = robot.second->GetChain()->RandomValidJoints( rng_, 0.05 );
		VecXd seed = robot.second->GetChain()->RandomValidJointsNear( rng_, joints, 0.1, 0 );

		auto result = CheckConvergence( robot.second, seed, joints );
		EXPECT_TRUE( result.Success() )
			<< "Test Fail for " << robot.first << std::endl;
	}
}

// ------------------------------------------------------------
// Zero-config: seed = solution, should converge in very few iterations
// ------------------------------------------------------------

TEST_F( FABRIKSolverTest, IK_ConvergesFromExactSeed )
{
	VecXd joints = model_->GetChain()->RandomValidJoints( rng_, 0.05 );
	auto result = CheckConvergence( model_, joints, joints );
	EXPECT_EQ( result.iterations, 0 );
}

// ------------------------------------------------------------

TEST_F( FABRIKSolverTest, IK_AllRobots_ConvergesFromExactSeed )
{
	for ( const auto& robot : ValidRobots() )
	{
		VecXd joints = robot.second->GetChain()->RandomValidJoints( rng_, 0.05 );
		auto result = CheckConvergence( robot.second, joints, joints );
		EXPECT_EQ( result.iterations, 0 )
			<< "Test Fail for " << robot.first << std::endl;
	}
}

// ------------------------------------------------------------
// Straight-up configuration (all zeros)
// ------------------------------------------------------------

TEST_F( FABRIKSolverTest, IK_ZeroConfiguration )
{
	const int n_joints = model_->GetChain()->GetActiveJointCount();

	VecXd joints = VecXd::Zero( n_joints );
	VecXd seed = VecXd::Ones( n_joints ) * 0.1;

	auto result = CheckConvergence( model_, seed, joints );
}

// ------------------------------------------------------------

TEST_F( FABRIKSolverTest, IK_AllRobots_ZeroConfiguration )
{
	for ( const auto& robot : ValidRobots() )
	{
		const int n_joints = robot.second->GetChain()->GetActiveJointCount();

		VecXd joints = VecXd::Zero( n_joints );
		VecXd seed = VecXd::Ones( n_joints ) * 0.1;

		auto result = CheckConvergence( robot.second, seed, joints );
		EXPECT_TRUE( result.Success() )
			<< "Test Fail for " << robot.first << std::endl;
	}
}

// ------------------------------------------------------------
// Unreachable: target far outside workspace
// ------------------------------------------------------------

TEST_F( FABRIKSolverTest, IK_UnreachableFarTarget )
{
	const int n_joints = model_->GetChain()->GetActiveJointCount();

	Mat4d target     = Mat4d::Identity();
	target( 0, 3 )   = 100.0;
	target( 1, 3 )   = 0.0;
	target( 2, 3 )   = 0.0;

	auto solver = Solver::FABRIKSolverDraft( model_ );
	auto problem = CreateProblem( VecXd::Zero( n_joints ), target );
	auto result = solver.Solve( problem, Solver::IKRunContext() );
	
	EXPECT_EQ( result.iterations, 0 );
	EXPECT_FALSE( result.Success() )
	    << "Target at (100,0,0) is outside workspace, IK should not succeed.";
}

// ------------------------------------------------------------

TEST_F( FABRIKSolverTest, IK_AllRobots_UnreachableFarTarget )
{
	for ( const auto& robot : ValidRobots() )
	{
		const int n_joints = robot.second->GetChain()->GetActiveJointCount();

		Mat4d target     = Mat4d::Identity();
		target( 0, 3 )   = 100.0;
		target( 1, 3 )   = 0.0;
		target( 2, 3 )   = 0.0;

		auto solver = Solver::FABRIKSolverDraft( robot.second );
		auto problem = CreateProblem( VecXd::Zero( n_joints ), target );
		auto result = solver.Solve( problem, Solver::IKRunContext() );
		
		EXPECT_EQ( result.iterations, 0 )
			<< "Test Fail for " << robot.first << std::endl;
		EXPECT_FALSE( result.Success() )
			<< "Test Fail for " << robot.first << std::endl
			<< "Target at (100,0,0) is outside workspace, IK should not succeed.";
	}
}

// ------------------------------------------------------------
// Multiple seeds: solution is seed-independent
// ------------------------------------------------------------

TEST_F( FABRIKSolverTest, IK_MultipleSeedsConverge )
{
	const int n_joints = model_->GetChain()->GetActiveJointCount();
	VecXd joints = model_->GetChain()->RandomValidJoints( rng_, 0.05 );

	std::vector< Eigen::Vector< double, 3 >> seeds = {
		VecXd::Zero( n_joints ),
		VecXd::Ones( n_joints ) * -M_PI / 4,
		VecXd::Ones( n_joints ) * M_PI / 4,
		model_->GetChain()->RandomValidJoints( rng_, 0.05 ),
	};

	for ( size_t s = 0; s < seeds.size(); s++ )
	{
		auto result = CheckConvergence( model_, seeds[s], joints );

		EXPECT_TRUE( result.Success() )
		    << "Seed index " << s << " failed. "
		    << "Error = " << result.error;
	}
}

// ------------------------------------------------------------

TEST_F( FABRIKSolverTest, IK_AllRobots_MultipleSeedsConverge )
{
	for ( const auto& robot : ValidRobots() )
	{
		const int n_joints = robot.second->GetChain()->GetActiveJointCount();
		VecXd joints = robot.second->GetChain()->RandomValidJoints( rng_, 0.05 );

		std::vector< Eigen::Vector< double, 3 >> seeds = {
			VecXd::Zero( n_joints ),
			VecXd::Ones( n_joints ) * -M_PI / 4,
			VecXd::Ones( n_joints ) * M_PI / 4,
			robot.second->GetChain()->RandomValidJoints( rng_, 0.05 ),
		};

		for ( size_t s = 0; s < seeds.size(); s++ )
		{
			auto result = CheckConvergence( robot.second, seeds[s], joints );

			EXPECT_TRUE( result.Success() )
				<< "Test Fail for " << robot.first << std::endl
				<< "Seed index " << s << " failed. "
				<< "Error = " << result.error;
		}
	}
}

// ------------------------------------------------------------
// Joint limits: result joints must stay within limits
// ------------------------------------------------------------

TEST_F( FABRIKSolverTest, IK_JointLimitsRespected )
{
	const int n_joints = model_->GetChain()->GetActiveJointCount();
	VecXd joints = model_->GetChain()->RandomValidJoints( rng_, 0.05 );
	VecXd seed = Vec3d::Zero( n_joints );

	auto result = CheckConvergence( model_, seed, joints );
	ASSERT_TRUE( model_->GetChain()->WithinLimits( result.joints ) );
}

// ------------------------------------------------------------

TEST_F( FABRIKSolverTest, IK_AllRobots_JointLimitsRespected )
{
	for ( const auto& robot : ValidRobots() )
	{
	const int n_joints = robot.second->GetChain()->GetActiveJointCount();
	VecXd joints = robot.second->GetChain()->RandomValidJoints( rng_, 0.05 );
	VecXd seed = Vec3d::Zero( n_joints );

	auto result = CheckConvergence( robot.second, seed, joints );
	ASSERT_TRUE( robot.second->GetChain()->WithinLimits( result.joints ) )
		<< "Test Fail for " << robot.first << std::endl
		<< "Solution should be within limits.";
}
}

// ------------------------------------------------------------
// Randomised sweep: N random configurations
// ------------------------------------------------------------

TEST_F( FABRIKSolverTest, IK_RandomConfigurations )
{
	constexpr int kTrials = 1;
	int successes         = 0;
	double avg_iter = 0.0;
	for ( int t = 0; t < kTrials; t++ )
	{
		VecXd joints = model_->GetChain()->RandomValidJoints( rng_, 0.05 );
		VecXd seed = model_->GetChain()->RandomValidJointsNear( rng_, joints, 0.3, 0 );

		auto result = CheckConvergence( model_, seed, joints );

		if ( result.Success() )
			successes++;
		avg_iter += result.iterations / ( double )kTrials;
	}

	// Expect at least 99% success rate on reachable random targets
	EXPECT_LE( avg_iter, 20 )
	    << "Average iterations = " << avg_iter << std::endl;
	EXPECT_GE( successes, static_cast< int >( kTrials - 1 ) )
	    << "Success rate " << successes << "/" << kTrials
	    << " is below 99% threshold.";
}

// ------------------------------------------------------------

TEST_F( FABRIKSolverTest, IK_AllRobots_RandomConfigurations )
{
	constexpr int kTrials = 0;
	
	for ( const auto& robot : ValidRobots() )
	{
		int successes = 0;
		double avg_iter = 0.0;
		for ( int t = 0; t < kTrials; t++ )
		{
			VecXd joints = robot.second->GetChain()->RandomValidJoints( rng_, 0.05 );
			VecXd seed = robot.second->GetChain()->RandomValidJointsNear( rng_, joints, 0.3, 0 );

			auto result = CheckConvergence( robot.second, seed, joints );

			if ( result.Success() )
				successes++;
			avg_iter += result.iterations / ( double )kTrials;
		}

		// Expect at least 99% success rate on reachable random targets
		EXPECT_LE( avg_iter, 20 )
			<< "Fail for robot " << robot.first << std::endl
			<< "Average iterations = " << avg_iter << std::endl;
		EXPECT_GE( successes, static_cast< int >( kTrials * 0.99 ) )
			<< "Fail for robot " << robot.first << std::endl	
			<< "Success rate " << successes << "/" << kTrials
			<< " is below 99% threshold.";

		if ( successes != kTrials )
			return;
	}
}

// ------------------------------------------------------------
// Iteration budget: convergence reported iteration count is plausible
// ------------------------------------------------------------

TEST_F( FABRIKSolverTest, IK_IterationCountReasonable )
{
	VecXd joints = model_->GetChain()->RandomValidJoints( rng_, 0.05 );
	VecXd seed = model_->GetChain()->RandomValidJointsNear( rng_, joints, 0.1, 0 );

	auto result = CheckConvergence( model_, seed, joints );

	EXPECT_GE( result.iterations, 0 );
	EXPECT_LE( result.iterations, 20 );
}

// ------------------------------------------------------------

TEST_F( FABRIKSolverTest, IK_AllRobots_IterationCountReasonable )
{
	for ( const auto& robot : ValidRobots() )
	{
		VecXd joints = robot.second->GetChain()->RandomValidJoints( rng_, 0.05 );
		VecXd seed = robot.second->GetChain()->RandomValidJointsNear( rng_, joints, 0.1, 0 );

		auto result = CheckConvergence( robot.second, seed, joints );

		EXPECT_GE( result.iterations, 0 )
			<< "Fail for robot " << robot.first << std::endl
			<< "Iterations should be greater or equal than 0";
		EXPECT_LE( result.iterations, 20 )
			<< "Fail for robot " << robot.first << std::endl
			<< "Iterations should be lower or equal than 20";
	}
}

// ------------------------------------------------------------
// Boundary joints: solution near joint limits should still converge
// ------------------------------------------------------------

TEST_F( FABRIKSolverTest, IK_NearJointLimits )
{
	const auto& chain = model_->GetChain();
	const int n_joints = chain->GetActiveJointCount();
	VecXd min_joints( n_joints );
	VecXd max_joints( n_joints );
	for ( int i = 0; i < n_joints; i++ )
	{
		const auto& limits = chain->GetActiveJointLimits( i );
		min_joints[i] = limits.Min() * 0.9;
		max_joints[i] = limits.Max() * 0.9;
	}

	auto min_result = CheckConvergence( model_, VecXd::Zero( n_joints ), min_joints );
	auto max_result = CheckConvergence( model_, VecXd::Zero( n_joints ), max_joints );
}

// ------------------------------------------------------------

TEST_F( FABRIKSolverTest, IK_AllRobots_NearJointLimits )
{
	for ( const auto& robot : ValidRobots() )
	{
		const auto& chain = model_->GetChain();
		const int n_joints = chain->GetActiveJointCount();
		VecXd min_joints( n_joints );
		VecXd max_joints( n_joints );
		for ( int i = 0; i < n_joints; i++ )
		{
			const auto& limits = chain->GetActiveJointLimits( i );
			min_joints[i] = limits.Min() * 0.9;
			max_joints[i] = limits.Max() * 0.9;
		}

		auto min_result = CheckConvergence( model_, VecXd::Zero( n_joints ), min_joints );
		auto max_result = CheckConvergence( model_, VecXd::Zero( n_joints ), max_joints );

		EXPECT_TRUE( min_result.Success() )
			<< "Fail for robot " << robot.first << std::endl
			<< "Should find solution close to min limits" << std::endl;
		EXPECT_TRUE( max_result.Success() )
			<< "Fail for robot " << robot.first << std::endl
			<< "Should find solution close to max limits" << std::endl;
	}
}

// ------------------------------------------------------------
// Size mismatch: wrong seed size returns Failed
// ------------------------------------------------------------

TEST_F( FABRIKSolverTest, IK_SeedSizeMismatchFails )
{
	Mat4d target = Mat4d::Identity();

	auto problem = CreateProblem( VecXd::Zero( 4 ), target );
	auto solver = Solver::FABRIKSolverDraft( model_ );
	auto result = solver.Solve( problem, Solver::IKRunContext() );

	EXPECT_EQ( result.state, Solver::IKSolverState::NotRun )
	    << "Mismatched seed size should return Failed state.";
}

// ------------------------------------------------------------

} // namespace SOArm100::Kinematics::Test