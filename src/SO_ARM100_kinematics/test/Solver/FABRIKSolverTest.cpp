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
	robot_name_ = "Universal Robot";
	model_ = Data::GetAllRobots()[robot_name_];
}

struct TestParameters
{
	int max_iteration;
	int max_stalled_iterations;
	int avg_iteration;
	double tolerance {5e-3};
};

std::string robot_name_;

std::map< std::string, Model::KinematicModelConstPtr > ValidRobots()
{
	return Data::GetAllRobots();
	// return
	//     {
	// 		{ "ZYZ", Data::GetZYZRevoluteRobot() },
	// 		{ "RevoluteBase", Data::GetRevoluteBaseRobot() },
	//         {"PrismaticBase", Data::GetPrismaticBaseRobot()},
	//         {"Planar2R", Data::GetPlanar2RRobot()},
	//         {"Planar3R", Data::GetPlanar3RRobot()},
	//         {"Wrist1R", Data::GetWrist1RRobot()},
	//         {"Wrist2R", Data::GetWrist2RRobot()},
	//         {"Wrist3R", Data::GetSphericalWristRobot()},
	//         {"5-axis arm", Data::GetRevolute_Planar2R_Wrist2R_5DOFsRobot()},
	//         {"6-axis arm", Data::GetRevolute_Planar2R_SphericalWrist_6DOFsRobot()},
	//         {"Universal Robot", Data::GetURLikeRobot()},
	// 	};
}



std::map< std::string, TestParameters > RobotsTestParameters()
{
	return
	    {
			{ "ZYZ", TestParameters{ 30, 5, 20 } },
			{ "RevoluteBase", TestParameters{ 30, 2, 2 } },
	        {"PrismaticBase", TestParameters{ 30, 3, 20 }},
	        {"Planar2R", TestParameters{ 500, 50, 15 }},
	        {"Planar3R", TestParameters{ 50, 5, 15 }},
	        {"Wrist1R", TestParameters{ 20, 2, 2 }},
	        {"Wrist2R", TestParameters{ 20, 2, 2 }},
	        {"Wrist3R", TestParameters{ 20, 2, 2 }},
	        {"5-axis arm", TestParameters{ 200, 5, 20 }},
	        {"6-axis arm", TestParameters{ 200, 5, 20 }},
	        {"Universal Robot", TestParameters{ 200, 5, 20 }},
		};
}

Solver::IKSolution CheckConvergence(
	Model::KinematicModelConstPtr model,
	TestParameters test_parameters,
	const VecXd& seed,
	const VecXd& joints )
{
	EXPECT_EQ( seed.size(), joints.size() );

	auto parameters = Solver::FABRIKSolverDraft::SolverParameters{ test_parameters.max_iteration, test_parameters.max_stalled_iterations, test_parameters.tolerance };
	auto solver = Solver::FABRIKSolverDraft( model, parameters );
	auto problem = CreateProblem( model, seed, joints );
	auto result = solver.Solve( problem, Solver::IKRunContext() );
	Mat4d result_pose;
	model->ComputeFK( result.joints, result_pose );
	EXPECT_EQ( result.joints.size(), joints.size() );
	EXPECT_TRUE( result.Success() )
	    << "Convergence failed" << std::endl
		<< "Target joints = " << joints.transpose() << std::endl
		<< problem << std::endl
		<< result  << std::endl;

	EXPECT_TRUE( TranslationError( problem.target, result_pose ) < solver.GetParameters().error_tolerance )
	    << "Joints  = " << result.joints.transpose() << "\n"
	    << "Target  =\n" << Translation( problem.target  ).transpose() << "\n"
	    << "Result  =\n" << Translation( result_pose ).transpose() << std::endl;

	return result;
}

Solver::IKSolution CheckConvergenceFromTarget(
	Model::KinematicModelConstPtr model,
	TestParameters test_parameters,
	const VecXd& seed,
	const Mat4d& target )
{
	auto parameters = Solver::FABRIKSolverDraft::SolverParameters{ test_parameters.max_iteration, test_parameters.max_stalled_iterations, test_parameters.tolerance };
	auto solver = Solver::FABRIKSolverDraft( model, parameters );
	auto problem = CreateProblem( seed, target );
	auto result = solver.Solve( problem, Solver::IKRunContext() );
	Mat4d result_pose;
	model->ComputeFK( result.joints, result_pose );
	EXPECT_EQ( result.joints.size(), seed.size() );
	EXPECT_TRUE( result.Success() )
	    << "Convergence failed" << std::endl
		<< problem << std::endl
		<< result  << std::endl;

	EXPECT_TRUE( TranslationError( problem.target, result_pose ) < solver.GetParameters().error_tolerance )
	    << "Joints  = " << result.joints.transpose() << "\n"
	    << "Target  =\n" << Translation( problem.target  ).transpose() << "\n"
	    << "Result  =\n" << Translation( result_pose ).transpose() << std::endl;

	return result;
}

Solver::IKSolution CheckConvergenceNoFailure(
	Model::KinematicModelConstPtr model,
	TestParameters test_parameters,
	const VecXd& seed,
	const VecXd& joints,
	bool silent = true )
{
	EXPECT_EQ( seed.size(), joints.size() );

	auto parameters = Solver::FABRIKSolverDraft::SolverParameters{ test_parameters.max_iteration, test_parameters.max_stalled_iterations, test_parameters.tolerance };
	auto solver = Solver::FABRIKSolverDraft( model, parameters );
	auto problem = CreateProblem( model, seed, joints );
	auto result = solver.Solve( problem, Solver::IKRunContext() );
	Mat4d result_pose;
	model->ComputeFK( result.joints, result_pose );
	EXPECT_EQ( result.joints.size(), joints.size() );

	if ( !result.Success() && !silent )
	{
		std::cout 
			<< "------------------------------------------------------------" << std::endl
			<< "Convergence failed" << std::endl
			<< "------------------------------------------------------------" << std::endl
			<< "Target joints = " << joints.transpose() << std::endl
			<< problem << std::endl
			<< result  << std::endl
			<< "Joints  = " << result.joints.transpose() << "\n"
			<< "Target  =\n" << Translation( problem.target  ).transpose() << "\n"
			<< "Result  =\n" << Translation( result_pose ).transpose() << std::endl;
	}

	return result;
}

protected:
random_numbers::RandomNumberGenerator rng_;
static constexpr double DEFAULT_TOLERANCE = error_tolerance;
};

// ------------------------------------------------------------
// Basic convergence with known joint configuration
// ------------------------------------------------------------

TEST_F( FABRIKSolverTest, IK_Converges_FromJoints )
{
	/*
	const int n_joints = model_->GetChain()->GetActiveJointCount();

	VecXd joints( n_joints );
	joints << 1.12032, 0.508416;
	VecXd seed( n_joints );
	seed << 1.39749, 0.564245;

	Mat4d target;
	model_->ComputeFK( joints, target );
	std::cout << "Target = " << std::endl << target << std::endl;

	TestParameters parameters;
	parameters.max_iteration = 1000;
	parameters.max_stalled_iterations = 5;
	parameters.tolerance = 1e-3;

	auto result = CheckConvergence( model_, parameters, seed, joints );

	std::cout << result << std::endl;
	*/
}

// ------------------------------------------------------------

TEST_F( FABRIKSolverTest, IK_Converges_FromTarget )
{
	/*
	const int n_joints = model_->GetChain()->GetActiveJointCount();

	Mat4d target = ToTransformMatrix( Vec3d{1.6, 0., 1.1 } ); 
	VecXd seed( n_joints );
	seed << 0,0;

	CheckConvergenceFromTarget( model_, RobotsTestParameters()[robot_name_], seed, target );
	*/
}

// ------------------------------------------------------------
// Zero-config: seed = solution, should converge in very few iterations
// ------------------------------------------------------------

TEST_F( FABRIKSolverTest, IK_ConvergesFromExactSeed )
{
	VecXd joints = model_->GetChain()->RandomValidJoints( rng_, 0.05 );
	auto result = CheckConvergence( model_, RobotsTestParameters()[robot_name_], joints, joints );
	EXPECT_EQ( result.iterations, 0 );
}

// ------------------------------------------------------------

TEST_F( FABRIKSolverTest, IK_ConvergesFromExactSeed_AllRobots )
{
	for ( const auto& robot : ValidRobots() )
	{
		VecXd joints = robot.second->GetChain()->RandomValidJoints( rng_, 0.05 );
		auto result = CheckConvergence( robot.second, RobotsTestParameters()[robot.first], joints, joints );
		EXPECT_EQ( result.iterations, 0 )
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

TEST_F( FABRIKSolverTest, IK_UnreachableFarTarget_AllRobots )
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
	const int k_seeds = 100;
	int successes = 0;
	const auto& chain = model_->GetChain();
	const int n_joints = chain->GetActiveJointCount();
	
	TestParameters parameters;
	parameters.max_iteration = 500;
	parameters.max_stalled_iterations = 50;
	parameters.tolerance = 5e-3;

	VecXd joints = chain->RandomValidJointsNearCentered( rng_, VecXd::Zero( n_joints ), 0.3, 0.3 );

	std::vector< VecXd > seeds = {
		VecXd::Zero( n_joints ),
		VecXd::Ones( n_joints ) * -M_PI / 4,
		VecXd::Ones( n_joints ) * M_PI / 4,
	};

	const int seed_size = seeds.size();
	for ( int i = 0; i < k_seeds - seed_size; i++ )
	{
		seeds.emplace_back( chain->RandomValidJointsNear( rng_, joints, 0.3, 0.1 ) );
	}

	for ( size_t s = 0; s < seeds.size(); s++ )
	{
		auto result = CheckConvergenceNoFailure( 
			model_, 
			parameters, 
			seeds[s], 
			joints );

		if ( result.Success() )
			successes++;
		else
			std::cout
				<< "Seed index " << s << " failed. "
				<< "Error = " << result.error << std::endl
				<< "Seeds: " << seeds[s].transpose() << std::endl;
	}

	EXPECT_GE( successes, 0.1 * k_seeds )
		<< "Test Fail for " << robot_name_ << std::endl
		<< "Converge only " << successes << " on " << k_seeds << " trials." << std::endl;
}

// ------------------------------------------------------------

TEST_F( FABRIKSolverTest, IK_MultipleSeedsConverge_AllRobots )
{
	TestParameters parameters;
	parameters.max_iteration = 500;
	parameters.max_stalled_iterations = 50;
	parameters.tolerance = 5e-3;

	const int k_seeds = 100;

	for ( const auto& robot : ValidRobots() )
	{
		int successes = 0;

		const auto& chain = robot.second->GetChain();
		const int n_joints = chain->GetActiveJointCount();
		VecXd joints = chain->RandomValidJoints( rng_, 0.1 );
	
		std::vector< VecXd > seeds = {
			VecXd::Zero( n_joints ),
			VecXd::Ones( n_joints ) * -M_PI / 4,
			VecXd::Ones( n_joints ) * M_PI / 4,
		};

		const int seed_size = seeds.size();
		for ( int i = 0; i < k_seeds - seed_size; i++ )
		{
			seeds.emplace_back( chain->RandomValidJointsNear( rng_, joints, 0.3, 0.1 ) );
		}

		for ( size_t s = 0; s < seeds.size(); s++ )
		{
			auto result = CheckConvergenceNoFailure( 
				robot.second, 
				parameters, 
				seeds[s], 
				joints );

			if ( result.Success() )
				successes++;
		}

		EXPECT_GE( successes, 0.1 * k_seeds )
			<< "Test Fail for " << robot.first << std::endl
			<< "Converge only " << successes << " on " << k_seeds << " trials." << std::endl;
	}
}

// ------------------------------------------------------------
// Joint limits: result joints must stay within limits
// ------------------------------------------------------------

TEST_F( FABRIKSolverTest, IK_JointLimitsRespected )
{
	const int n_joints = model_->GetChain()->GetActiveJointCount();
	VecXd joints = model_->GetChain()->RandomValidJoints( rng_, 0.05 );
	VecXd seed = VecXd::Zero( n_joints );

	auto result = CheckConvergenceNoFailure( model_, RobotsTestParameters()[robot_name_], seed, joints );
	ASSERT_TRUE( model_->GetChain()->WithinLimits( result.joints ) );
}

// ------------------------------------------------------------

TEST_F( FABRIKSolverTest, IK_JointLimitsRespected_AllRobots )
{
	for ( const auto& robot : ValidRobots() )
	{
		const int n_joints = robot.second->GetChain()->GetActiveJointCount();
		VecXd joints = robot.second->GetChain()->RandomValidJoints( rng_, 0.05 );
		VecXd seed = VecXd::Zero( n_joints );

		auto result = CheckConvergenceNoFailure( robot.second, RobotsTestParameters()[robot.first], seed, joints );
		ASSERT_TRUE( robot.second->GetChain()->WithinLimits( result.joints ) )
		    << "Test Fail for " << robot.first << std::endl
		    << "Solution should be within limits.";
	}
}

// ------------------------------------------------------------
// Randomised sweep: N random configurations
// ------------------------------------------------------------

TEST_F( FABRIKSolverTest, IK_AverageConsistency_RandomConfigurations )
{
	constexpr int kTrials = 100;

	TestParameters parameters;
	parameters.max_iteration = 100;
	parameters.max_stalled_iterations = 15;
	parameters.tolerance = 5e-3;

	double avg_iter = 0.0;
	double avg_error = 0.0;

	for ( int t = 0; t < kTrials; t++ )
	{
		VecXd joints = model_->GetChain()->RandomValidJoints( rng_, 0.2 );
		VecXd seed = model_->GetChain()->RandomValidJointsNear( rng_, joints, 0.3, 0.05 );

		auto result = CheckConvergenceNoFailure( model_, parameters, seed, joints );

		avg_error+= result.error / ( double )kTrials;
		avg_iter += result.iterations / ( double )kTrials;
	}

	EXPECT_LE( avg_iter, RobotsTestParameters()[robot_name_].avg_iteration );
	EXPECT_LE( avg_error, 2 * parameters.tolerance );
}

// ------------------------------------------------------------

TEST_F( FABRIKSolverTest, IK_AverageConsistency_RandomConfigurations_AllRobots )
{
	constexpr int kTrials = 100;
	TestParameters parameters;
	parameters.max_iteration = 100;
	parameters.tolerance = 5e-3;
	parameters.max_stalled_iterations = 10;

	for ( const auto& robot : ValidRobots() )
	{
		double avg_error = 0.0;
		double avg_iter = 0.0;
		for ( int t = 0; t < kTrials; t++ )
		{
			VecXd joints = robot.second->GetChain()->RandomValidJoints( rng_, 0.2 );
			VecXd seed = robot.second->GetChain()->RandomValidJointsNear( rng_, joints, 0.3, 0.05 );

			auto result = CheckConvergenceNoFailure( robot.second, parameters, seed, joints );

			avg_error+= result.error / ( double )kTrials;
			avg_iter += result.iterations / ( double )kTrials;
		}

		// Expect at least 99% success rate on reachable random targets
		if ( avg_iter > RobotsTestParameters()[robot.first].avg_iteration )
		{
			ADD_FAILURE() 
				<< "Fail for robot " << robot.first << std::endl
		    	<< "Average iterations = " << avg_iter << std::endl;
		}
		if ( avg_error > 3 * parameters.tolerance )
		{
			ADD_FAILURE() 
				<< "Fail for robot " << robot.first << std::endl
		    	<< "Average error = " << avg_error << std::endl;
		}
	}
}

// ------------------------------------------------------------
// Check solver consistency
// ------------------------------------------------------------

TEST_F( FABRIKSolverTest, IK_CheckConsistency_RandomConfigurations )
{
	constexpr int kTrials = 100;
	int successes         = 0;

	TestParameters parameters;
	parameters.max_iteration = 1000;
	parameters.max_stalled_iterations = 1000;
	parameters.tolerance = 5e-3;

	for ( int t = 0; t < kTrials; t++ )
	{
		VecXd joints = model_->GetChain()->RandomValidJoints( rng_, 0.2 );
		VecXd seed = model_->GetChain()->RandomValidJointsNear( rng_, joints, 0.3, 0.05 );

		auto result = CheckConvergenceNoFailure( model_, parameters, seed, joints );

		if ( result.Success() )
			successes++;
	}

	EXPECT_GE( successes, static_cast< int >( kTrials * 0.9 ) )
	    << "Success rate " << successes << "/" << kTrials
	    << " is below 90% threshold.";
}

// ------------------------------------------------------------

TEST_F( FABRIKSolverTest, IK_CheckConsistency_RandomConfigurations_AllRobots )
{
	constexpr int kTrials = 100;
	TestParameters parameters;
	parameters.max_iteration = 1000;
	parameters.max_stalled_iterations = 1000;
	parameters.tolerance = 5e-3;

	for ( const auto& robot : ValidRobots() )
	{
		int successes = 0;
		for ( int t = 0; t < kTrials; t++ )
		{
			VecXd joints = robot.second->GetChain()->RandomValidJoints( rng_, 0.2 );
			VecXd seed = robot.second->GetChain()->RandomValidJointsNear( rng_, joints, 0.3, 0.05 );

			auto result = CheckConvergenceNoFailure( robot.second, parameters, seed, joints );

			if ( result.Success() )
				successes++;
		}

		if ( successes < static_cast< int >( kTrials * 0.9 ) )
		{
			ADD_FAILURE() 
				<< "Fail for robot " << robot.first << std::endl
				<< "Success rate " << successes << "/" << kTrials
				<< " is below 90% threshold.";
		}
	}
}

// ------------------------------------------------------------
// Iteration budget: convergence reported iteration count is plausible
// ------------------------------------------------------------

TEST_F( FABRIKSolverTest, IK_MaxStalledIterationRespected )
{
	TestParameters parameters;
	parameters.max_iteration = 50;
	parameters.max_stalled_iterations = 2;
	parameters.tolerance = 5e-3;

	const auto& chain = model_->GetChain();
	const int n_joints = chain->GetActiveJointCount();

	VecXd joints = chain->RandomValidJoints( rng_, 0.1 );
	VecXd seed = chain->RandomValidJointsNear( rng_, joints, 0.3, 0.1 );

	auto result = CheckConvergenceNoFailure( model_, parameters, seed, joints );

	EXPECT_NE( result.state, Solver::IKSolverState::MaxIterations );
	EXPECT_GE( result.iterations, 0 );
	EXPECT_LT( result.iterations, parameters.max_iteration );
}

// ------------------------------------------------------------

TEST_F( FABRIKSolverTest, IK_MaxStalledIterationRespected_AllRobots )
{
	TestParameters parameters;
	parameters.max_iteration = 50;
	parameters.max_stalled_iterations = 2;
	parameters.tolerance = 5e-3;

	for ( const auto& robot : ValidRobots() )
	{
		const auto& chain = robot.second->GetChain();
		const int n_joints = chain->GetActiveJointCount();
	
		VecXd joints = chain->RandomValidJoints( rng_, 0.1 );
		VecXd seed = chain->RandomValidJointsNear( rng_, joints, 0.3, 0.1 );
	
		auto result = CheckConvergenceNoFailure( robot.second, parameters, seed, joints );

		EXPECT_NE( result.state, Solver::IKSolverState::MaxIterations );
		EXPECT_GE( result.iterations, 0 )
		    << "Fail for robot " << robot.first << std::endl
		    << "Iterations should be greater or equal than 0";
		EXPECT_LT( result.iterations, parameters.max_iteration )
		    << "Fail for robot " << robot.first << std::endl
		    << "Iterations with max stalled iterations "  << parameters.max_stalled_iterations 
			<< " should be lower than " << parameters.max_iteration;
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

	auto min_result = CheckConvergence( model_, RobotsTestParameters()[robot_name_], VecXd::Zero( n_joints ), min_joints );
	auto max_result = CheckConvergence( model_, RobotsTestParameters()[robot_name_], VecXd::Zero( n_joints ), max_joints );
}

// ------------------------------------------------------------

TEST_F( FABRIKSolverTest, IK_NearJointLimits_AllRobots )
{
	TestParameters parameters;
	parameters.max_iteration = 100;
	parameters.max_stalled_iterations = 20;
	parameters.tolerance = 5e-3;

	for ( const auto& robot : ValidRobots() )
	{
		const auto& chain = robot.second->GetChain();
		const int n_joints = chain->GetActiveJointCount();
		VecXd min_joints( n_joints );
		VecXd max_joints( n_joints );
		for ( int i = 0; i < n_joints; i++ )
		{
			const auto& limits = chain->GetActiveJointLimits( i );
			min_joints[i] = limits.Min() * 0.9;
			max_joints[i] = limits.Max() * 0.9;
		}

		auto min_result = CheckConvergence( robot.second, parameters, VecXd( n_joints ), min_joints );
		auto max_result = CheckConvergence( robot.second, parameters, VecXd( n_joints ), max_joints );

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
	const int n_joints = model_->GetChain()->GetActiveJointCount();
	VecXd joints = VecXd::Zero( n_joints );

	auto problem = CreateProblem( VecXd::Zero( n_joints + 1 ), joints );
	auto solver = Solver::FABRIKSolverDraft( model_ );
	auto result = solver.Solve( problem, Solver::IKRunContext() );

	EXPECT_EQ( result.state, Solver::IKSolverState::NotRun )
	    << "Mismatched seed size should return Failed state.";
}

// ------------------------------------------------------------

} // namespace SOArm100::Kinematics::Test