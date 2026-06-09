#include "Heuristic/TopologyHeuristic.hpp"

#include "Global.hpp"

#include "Model/Joint/JointGroup.hpp"
#include "RobotModelTestData.hpp"
#include "KinematicTestBase.hpp"

#include "Heuristic/IKHeuristicState.hpp"
#include "Heuristic/IKPresolution.hpp"
#include "Solver/IKProblem.hpp"
#include "Model/KinematicModel.hpp"
#include "Solver/IKRunContext.hpp"
#include "Utils/KinematicsUtils.hpp"
#include "Utils/StringConverter.hpp"

#include <gtest/gtest.h>
#include <limits>
#include <random_numbers/random_numbers.h>
#include <map>

namespace SOArm100::Kinematics::Test
{

// ------------------------------------------------------------
// Fixture
// ------------------------------------------------------------

class TopologyHeuristicTest : public KinematicTestBase
{
protected:
void SetUp() override
{
	// robot_name_ = "ZYZ";
	// robot_name_ = "RevoluteBase";
	// robot_name_ = "PrismaticBase";
	// robot_name_ = "Planar2R";
	// robot_name_ = "Planar3R";
	// robot_name_ = "Wrist1R";
	// robot_name_ = "Wrist2R";
	// robot_name_ = "Wrist3R";
	// robot_name_ = "5-axis arm";
	// robot_name_ = "6-axis arm";
	robot_name_ = "Universal Robot";

	model_ = Data::GetAllRobots()[robot_name_];
}

std::string robot_name_;
random_numbers::RandomNumberGenerator rng_;

std::map< std::string, Model::KinematicModelConstPtr > ValidRobots()
{
	return Data::GetAllRobots();
}

Heuristic::IKPresolution CheckPresolve(
	Model::KinematicModelConstPtr model,
	std::string robot_name,
	const VecXd& seed,
	const VecXd& joints ) const
{
	EXPECT_EQ( seed.size(), joints.size() );

	auto heuristic = Heuristic::TopologyHeuristic( model );
	auto problem = CreateProblem( model, seed, joints );

	auto result = heuristic.Presolve( problem, Solver::IKRunContext() );

	EXPECT_EQ( result.joints.size(), joints.size() );
	Mat4d result_pose;
	model->ComputeFK( result.joints, result_pose );
	double error = PoseError( model, problem, result );

	EXPECT_TRUE( IsApprox( problem.target, result_pose, 1e-3 ) )
	    << "Fail for robot " << robot_name << std::endl
	    << "Target = \n" << problem.target << std::endl
	    << "Result = \n" << result_pose << std::endl
	    << "Position Error = \n" << TranslationError( problem.target, result_pose ) << std::endl
	    << "Rotation Error = \n" << RotationError( problem.target, result_pose ) << std::endl
	    << "Pose     Error = \n" << error << std::endl
	    << "Joints = \n" << joints << std::endl
	    << problem << std::endl
	    << result << std::endl;

	return result;
}

Heuristic::IKPresolution CheckPresolveNoFailure(
	Model::KinematicModelConstPtr model,
	std::string robot_name,
	const VecXd& seed,
	const VecXd& joints,
	bool pSilent = true ) const
{
	EXPECT_EQ( seed.size(), joints.size() );

	auto heuristic = Heuristic::TopologyHeuristic( model );
	auto problem = CreateProblem( model, seed, joints );

	auto result = heuristic.Presolve( problem, Solver::IKRunContext() );

	EXPECT_EQ( result.joints.size(), joints.size() );
	Mat4d result_pose;
	model->ComputeFK( result.joints, result_pose );
	double error = PoseError( model, problem, result );

	if ( !IsApprox( problem.target, result_pose, 1e-3 ) && !pSilent )
	{
		std::cout
		    << "Fail for robot " << robot_name << std::endl
		    << "Target = \n" << problem.target << std::endl
		    << "Result = \n" << result_pose << std::endl
		    << "Position Error = \n" << TranslationError( problem.target, result_pose ) << std::endl
		    << "Rotation Error = \n" << RotationError( problem.target, result_pose ) << std::endl
		    << "Pose     Error = \n" << error << std::endl
		    << "Joints = \n" << joints << std::endl
		    << problem << std::endl
		    << result << std::endl;
	}

	return result;
}

Heuristic::IKPresolution CheckPresolveFromTarget(
	Model::KinematicModelConstPtr model,
	std::string robot_name,
	const VecXd& seed,
	const Mat4d& target )
{
	auto heuristic = Heuristic::TopologyHeuristic( model );
	auto problem = CreateProblem( seed, target );
	auto result = heuristic.Presolve( problem, Solver::IKRunContext() );

	EXPECT_EQ( result.joints.size(), seed.size() );
	Mat4d result_pose;
	model->ComputeFK( result.joints, result_pose );
	double error = PoseError( model, problem, result );

	EXPECT_TRUE( IsApprox( problem.target, result_pose, 1e-3 ) )
	    << "Fail for robot " << robot_name << std::endl
	    << "Target = \n" << problem.target << std::endl
	    << "Result = \n" << result_pose << std::endl
	    << "Position Error = \n" << TranslationError( problem.target, result_pose ) << std::endl
	    << "Rotation Error = \n" << RotationError( problem.target, result_pose ) << std::endl
	    << "Pose     Error = \n" << error << std::endl
	    << result << std::endl;

	return result;
}

};

// ------------------------------------------------------------
// Basic execution with known joint configuration
// ------------------------------------------------------------

TEST_F( TopologyHeuristicTest, Presolve_Converges_FromJoints )
{
	const int n_joints = model_->GetChain()->GetActiveJointCount();

	VecXd joints = VecXd::Zero( n_joints ); // = model_->GetChain()->RandomValidJoints( rng_, 0.05 );
	VecXd seed = VecXd::Ones( n_joints ) * M_PI; // = model_->GetChain()->RandomValidJointsNear( rng_, joints, 0.3, 0.05 );
	joints << -0.186106,   2.58615, -0.557316,   1.37537,  0.354778, -0.786451;

	auto result = CheckPresolve( model_, robot_name_, seed, joints );

	// Topology heuristic chains partial successes, so we expect at least a partial success
	// for a reachable target near the seed.
	EXPECT_TRUE( result.PartialOrSuccess() )
	    << "Presolve failed completely for a valid target." << std::endl;
}

// ------------------------------------------------------------

TEST_F( TopologyHeuristicTest, Presolve_Converges_FromJoints_AllRobots )
{
	for ( const auto& robot : ValidRobots() )
	{
		const int n_joints = robot.second->GetChain()->GetActiveJointCount();

		VecXd joints = robot.second->GetChain()->RandomValidJoints( rng_, 0.05 );
		VecXd seed = robot.second->GetChain()->RandomValidJointsNear( rng_, joints, 0.3, 0.05 );

		auto result = CheckPresolveNoFailure( robot.second, robot.first, seed, joints, false );

		EXPECT_TRUE( result.PartialOrSuccess() )
		    << "Test Fail for " << robot.first << std::endl
		    << "Presolve should return Success or PartialSuccess for reachable targets.";
	}
}

// ------------------------------------------------------------
// Presolve consistency
// ------------------------------------------------------------

TEST_F( TopologyHeuristicTest, Presolve_Consistency )
{
	const auto& chain = model_->GetChain();
	const int ITER = 10000;
	double avg_iterations = 0.0;
	double avg_non_success_error = 0.0;
	double max_non_success_error = 0.0;
	double avg_error = 0.0;
	int k_successes = 0;
	for ( int i = 0; i < ITER; i++ )
	{
		// Target joints
		VecXd joints = chain->RandomValidJoints( rng_, 0 );

		// Seed joints
		VecXd seed = chain->RandomValidJointsNear( rng_, joints, 0.3 );

		auto problem = CreateProblem( model_, seed, joints );
		auto result = CheckPresolveNoFailure( model_, robot_name_, seed, joints );

		auto result_pose = ComputeFK( model_, result.joints );
		// Check that the solution is valid
		if ( !result.Success() )
		{
			avg_non_success_error += result.error;
			max_non_success_error = std::max( max_non_success_error, result.error );
			// std::cout << "Joints " << joints.transpose() << std::endl;
			// std::cout << "Problem" << std::endl << problem << std::endl
			// 		  << "Result" << std::endl << result << std::endl;
		}
		else
		{
			EXPECT_EQ( result.joints.size(), model_->GetChain()->GetActiveJointCount() ) << "Result should contain values for all joints";
			// EXPECT_TRUE( IsApprox( problem.target, result_pose, rotation_tolerance, translation_tolerance ) )
			// 	<< "Target=\n" << problem.target << std::endl
			// 	<< "Result=\n" << result_pose << std::endl
			// 	<< "Result joints= " << result.joints.transpose() << std::endl;

			k_successes++;
		}

		avg_iterations += result.iterations / ( double )ITER;
		avg_error += result.error / ( double )ITER;
	}

	avg_non_success_error = k_successes != ITER ? avg_non_success_error / ( double )( ITER - k_successes ) : 0.0;
	EXPECT_LE( avg_error, 5 * error_tolerance );
	EXPECT_LT( max_non_success_error, std::numeric_limits< double >::infinity() );
	EXPECT_LE( avg_iterations, 5 );

	std::cout << "=========== Robot " << robot_name_ << " ===========\n";
	std::cout << "Avg iter    = " << avg_iterations << std::endl;
	std::cout << "Avg error   = " << avg_error << std::endl;
	std::cout << "Avg fail err= " << avg_non_success_error << std::endl;
	std::cout << "Max fail err= " << max_non_success_error << std::endl;
	std::cout << "k_successes = " << k_successes << " / " << ITER << std::endl;
}

// ------------------------------------------------------------

TEST_F( TopologyHeuristicTest, Presolve_Consistency_AllRobots )
{
	const int ITER = 100;

	for ( const auto& robot : ValidRobots() )
	{
		const auto& chain = robot.second->GetChain();
		double avg_iterations = 0.0;
		double avg_non_success_error = 0.0;
		double max_non_success_error = 0.0;
		double avg_error = 0.0;
		int k_successes = 0;
		for ( int i = 0; i < ITER; i++ )
		{
			VecXd joints = chain->RandomValidJoints( rng_, 0 );
			VecXd seed = chain->RandomValidJointsNear( rng_, joints, 0.3 );

			auto problem = CreateProblem( robot.second, seed, joints );
			auto result = CheckPresolveNoFailure( robot.second, robot.first, seed, joints );

			auto result_pose = ComputeFK( robot.second, result.joints );
			if ( !result.PartialOrSuccess() || result.error > 15 * problem.tolerance )
			{
				avg_non_success_error += result.error;
				max_non_success_error = std::max( max_non_success_error, result.error );
			}
			else
			{
				EXPECT_EQ( result.joints.size(), robot.second->GetChain()->GetActiveJointCount() ) << "Result should contain values for all joints";

				avg_iterations += result.iterations / ( double )ITER;
				avg_error += result.error / ( double )ITER;
			}
		}

		EXPECT_LE( avg_error, 5 * error_tolerance );
		EXPECT_LT( max_non_success_error, std::numeric_limits< double >::infinity() );
		EXPECT_LE( avg_iterations, 5 );
	}
}

// ------------------------------------------------------------
// Zero-config: seed = solution, should return success immediately
// ------------------------------------------------------------

TEST_F( TopologyHeuristicTest, Presolve_ConvergesFromExactSeed )
{
	VecXd joints = model_->GetChain()->RandomValidJoints( rng_, 0.05 );
	auto result = CheckPresolve( model_, robot_name_, joints, joints );

	EXPECT_EQ( result.state, Heuristic::IKHeuristicState::Success );
	EXPECT_EQ( result.iterations, 0 );
}

// ------------------------------------------------------------

TEST_F( TopologyHeuristicTest, Presolve_ConvergesFromExactSeed_AllRobots )
{
	for ( const auto& robot : ValidRobots() )
	{
		VecXd joints = robot.second->GetChain()->RandomValidJoints( rng_, 0.05 );
		auto result = CheckPresolve( robot.second, robot.first, joints, joints );

		// At worst, an exact seed should cascade through heuristics and yield partial/success
		EXPECT_TRUE( result.PartialOrSuccess() )
		    << "Test Fail for " << robot.first << std::endl;
	}
}

// ------------------------------------------------------------
// Unreachable: target far outside workspace
// ------------------------------------------------------------

TEST_F( TopologyHeuristicTest, Presolve_UnreachableFarTarget )
{
	const int n_joints = model_->GetChain()->GetActiveJointCount();

	Mat4d target     = Mat4d::Identity();
	target( 0, 3 )   = 100.0;
	target( 1, 3 )   = 0.0;
	target( 2, 3 )   = 0.0;

	VecXd seed = VecXd::Zero( n_joints );

	auto heuristic = Heuristic::TopologyHeuristic( model_ );
	auto problem = CreateProblem( seed, target );
	auto result = heuristic.Presolve( problem, Solver::IKRunContext() );

	EXPECT_EQ( result.state, Heuristic::IKHeuristicState::Fail )
	    << "Target at (100,0,0) is outside workspace, heuristic should not claim full success.";
}

// ------------------------------------------------------------

TEST_F( TopologyHeuristicTest, Presolve_UnreachableFarTarget_AllRobots )
{
	for ( const auto& robot : ValidRobots() )
	{
		const int n_joints = robot.second->GetChain()->GetActiveJointCount();

		Mat4d target     = Mat4d::Identity();
		target( 0, 3 )   = 100.0;
		target( 1, 3 )   = 0.0;
		target( 2, 3 )   = 0.0;

		VecXd seed = VecXd::Zero( n_joints );

		auto heuristic = Heuristic::TopologyHeuristic( robot.second );
		auto problem = CreateProblem( seed, target );
		auto result = heuristic.Presolve( problem, Solver::IKRunContext() );

		EXPECT_EQ( result.state, Heuristic::IKHeuristicState::Fail )
		    << "Test Fail for " << robot.first << std::endl
		    << "Target at (100,0,0) is outside workspace, heuristic should not claim full success.";
	}
}

// ------------------------------------------------------------
// Cascading states: ensure base -> intermediate -> wrist pass properly
// ------------------------------------------------------------

TEST_F( TopologyHeuristicTest, Presolve_MaintainsValidJointVectorSize )
{
	// Regardless of failure or partial success, the output joint vector
	// must always match the size of the active joint chain.
	for ( const auto& robot : ValidRobots() )
	{
		const int n_joints = robot.second->GetChain()->GetActiveJointCount();

		VecXd joints = robot.second->GetChain()->RandomValidJoints( rng_, 0.2 );
		VecXd seed = VecXd::Zero( n_joints );

		auto heuristic = Heuristic::TopologyHeuristic( robot.second );
		auto problem = CreateProblem( robot.second, seed, joints );
		auto result = heuristic.Presolve( problem, Solver::IKRunContext() );

		EXPECT_EQ( result.joints.size(), n_joints )
		    << "Test Fail for " << robot.first << std::endl
		    << "TopologyHeuristic corrupted the joint vector size during cascades.";
	}
}

// ------------------------------------------------------------

} // namespace SOArm100::Kinematics::Test