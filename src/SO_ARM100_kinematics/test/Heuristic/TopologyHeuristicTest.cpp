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
	robot_name_ = "RevoluteBase";
	// robot_name_ = "5-axis arm";
	// robot_name_ = "6-axis arm";
	// robot_name_ = "Universal Robot";
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
	VecXd seed = VecXd::Ones( n_joints ) * -M_PI; // = model_->GetChain()->RandomValidJointsNear( rng_, joints, 0.3, 0.05 );
	joints[0] = -M_PI / 3;
	joints[1] = -M_PI / 4;
	// joints[2] = M_PI / 6;
	// joints[3] = M_PI / 6;
	// joints[4] = -M_PI / 12;
	// joints[5] = -M_PI / 6;

	auto result = CheckPresolve( model_, robot_name_, seed, joints );

	// VecXd first_joints = VecXd::Zero( n_joints );
	// first_joints[0] = result.joints[0];
	// first_joints[1] = result.joints[1];
	// first_joints[2] = result.joints[2];
	// first_joints[3] = result.joints[3];
	// Mat4d tcp_in_wrist = model_->GetTopology().Get( Model::planarNR_name )->tip_home;
	// std::cout << "Result Joints " << std::endl << result.joints << std::endl;
	// std::cout << "Target WC =" << std::endl;
	// Mat4d target = ComputeFK( model_, joints );
	// std::cout << Translation( target ) - Rotation( target ) * Translation( tcp_in_wrist ) << std::endl;
	// std::cout << "Result WC =" << std::endl;
	// Mat4d result_pose = ComputeFK( model_, first_joints );
	// std::cout << Translation( result_pose ) - Rotation( target ) * Translation( tcp_in_wrist ) << std::endl;

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