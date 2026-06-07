#include "PipelineSolver/PipelineSolver.hpp"

#include "Model/Joint/JointGroup.hpp"
#include "RobotModelTestData.hpp"

#include "Global.hpp"

#include "DLS/DLSSolver.hpp"
#include "Heuristic/TopologyHeuristic.hpp"
#include "KinematicTestBase.hpp"
#include "Model/KinematicModel.hpp"
#include "PipelineSolver/IKPipeline.hpp"
#include "PipelineSolver/PipelineBuilder.hpp"
#include "Scorer/CloseToCenterScorer.hpp"
#include "Scorer/CloseToSeedScorer.hpp"
#include "Scorer/IKSolutionScorer.hpp"
#include "Scorer/ManipulabilityScorer.hpp"
#include "Scorer/PoseErrorScorer.hpp"
#include "Scorer/WeightedScorersBuilder.hpp"
#include "Seed/IKOppositeSeedGenerator.hpp"
#include "Solver/IKRunContext.hpp"
#include "Solver/IKSolution.hpp"
#include "Utils/KinematicsUtils.hpp"
#include "Utils/StringConverter.hpp"

#include <Eigen/src/Core/Matrix.h>
#include <cmath>
#include <gtest/gtest.h>
#include <memory>
#include <vector>

namespace SOArm100::Kinematics::Test
{

// ------------------------------------------------------------
// Pipeline Solver Test
// ------------------------------------------------------------

class PipelineSolverTest : public KinematicTestBase
{
protected:
void SetUp() override
{
	model_ = Data::GetURLikeRobot();

	std::vector< std::unique_ptr< const Solver::IKPipeline >> pipelines;

	Solver::DLSSolver::SolverParameters params_safe_solver;
	params_safe_solver.max_iterations = 500;
	params_safe_solver.max_stalle_iterations = 2;
	Solver::DLSSolver::SolverParameters params_fast_solver;
	params_fast_solver.max_iterations = 100;
	params_fast_solver.max_stalle_iterations = 2;

	pipelines.emplace_back(
		Solver::PipelineBuilder{}
		.WithHeuristic( std::make_unique< Heuristic::TopologyHeuristic >( model_ ) )
		.WithSolver( std::make_unique< Solver::DLSSolver >( model_, params_fast_solver ) )
		.Build() );

	pipelines.emplace_back(
		Solver::PipelineBuilder{}
		.WithSeedGenerator( std::make_unique< Seed::IKOppositeSeedGenerator >( model_, *model_->GetTopology().Get( Model::revolute_base_name ) ) )
		.WithHeuristic( std::make_unique< Heuristic::TopologyHeuristic >( model_ ) )
		.WithSolver( std::make_unique< Solver::DLSSolver >( model_, params_fast_solver ) )
		.Build() );

	pipelines.emplace_back(
		Solver::PipelineBuilder{}
		.WithSeedGenerator( std::make_unique< Seed::IKOppositeSeedGenerator >( model_, *model_->GetTopology().Get( Model::planarNR_name ) ) )
		.WithHeuristic( std::make_unique< Heuristic::TopologyHeuristic >( model_ ) )
		.WithSolver( std::make_unique< Solver::DLSSolver >( model_, params_fast_solver ) )
		.Build() );

	pipelines.emplace_back(
		Solver::PipelineBuilder{}
		.WithSolver( std::make_unique< Solver::DLSSolver >( model_, params_safe_solver ) )
		.Build() );

	// Solver::DLSSolver::SolverParameters params_random_start_solver;
	// params_safe_solver.max_iterations = 1000;
	// params_safe_solver.max_stalle_iterations = 1;

	// pipelines.emplace_back(
	// 	Solver::PipelineBuilder{}
	// 	.WithSeedGenerator( std::make_unique< Seed::IKRandomSeedGenerator >( model_ ) )
	// 	.WithSolver( std::make_unique< Solver::DLSSolver >( model_, params_random_start_solver ) )
	// 	.Build() );

	auto scorer = Scorer::WeightedScorersBuilder{}
	.Add( 1.0, std::make_unique< Scorer::CloseToCenterScorer >( model_ ) )
	.Add( 3.0, std::make_unique< Scorer::CloseToSeedScorer >( model_ ) )
	.Add( 3.0, std::make_unique< Scorer::ManipulabilityScorer >( model_ ) )
	.Add( 1.0, std::make_unique< Scorer::PoseErrorScorer >( model_, Scorer::PoseErrorScorer::ScorerParameters() ) )
	.Build();
	// Avg center = 0.55, Std center = 0.06
	// Avg seed   = 0.07, Std seed   = 0.004
	// Avg manip  = 0.1, Std manip  = 0.06
	// Avg error  = 0.00, Std error  = 0.00

	Solver::PipelineSolverParameters parameters;
	parameters.strategy = Solver::PipelineCompletionStrategy::WaitForAcceptableResult;
	parameters.min_score_threshold = 0.2;

	// Create a solver
	solver_ = std::unique_ptr< Solver::PipelineSolver >( new Solver::PipelineSolver(
															 model_,
															 std::move( pipelines ),
															 std::move( scorer ),
															 parameters ) );
}

void TearDown() override
{
}

std::unique_ptr< Solver::PipelineSolver > solver_;
random_numbers::RandomNumberGenerator rng_;
};

// ------------------------------------------------------------
// ------------------------------------------------------------

TEST_F( PipelineSolverTest, InverseKinematic_Success )
{
	// Target joints
	VecXd joints( 5 );
	joints << M_PI, M_PI / 2, M_PI / 3, M_PI / 4, M_PI / 5;

	// Seed joints
	VecXd seed( 5 );
	seed << 0.1, 0.1, 0.1, 0.1, 0.1;

	auto problem = CreateProblem( model_, seed, joints );
	auto result = solver_->Solve( problem, Solver::IKRunContext() );

	auto result_pose = ComputeFK( model_, result.joints );

	// Check that the solution is valid
	EXPECT_TRUE( result.Success() ) << "IK should succeed for reachable target";
	EXPECT_EQ( result.joints.size(), model_->GetChain()->GetActiveJointCount() ) << "Result should contain values for all joints";
	EXPECT_TRUE( IsApprox( problem.target, result_pose ) )
	    << "Problem =\n" << problem << std::endl
	    << "Result  =\n" << result << std::endl;
}

// ------------------------------------------------------------

TEST_F( PipelineSolverTest, InverseKinematic_Consistency )
{
	const auto& chain = model_->GetChain();
	const int ITER = 100;
	double avg_iterations = 0.0;
	double avg_score = 0.0;
	double var_score = 0.0;
	double avg_non_success_error = 0.0;
	double max_non_success_error = 0.0;
	double avg_error = 0.0;
	double delta = 0.0;
	int k_successes = 0;
	for ( int i = 0; i < ITER; i++ )
	{
		// Target joints
		VecXd joints = chain->RandomValidJoints( rng_, 0 );

		// Seed joints
		VecXd seed = chain->RandomValidJointsNear( rng_, joints, 0.3 );

		auto problem = CreateProblem( model_, seed, joints );
		auto result = solver_->Solve( problem, Solver::IKRunContext() );

		auto result_pose = ComputeFK( model_, result.joints );
		// Check that the solution is valid
		if ( !result.Success() )
		{
			avg_non_success_error += result.error;
			max_non_success_error = std::max( max_non_success_error, result.error );
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
			avg_iterations += result.iterations / ( double )ITER;

			delta += result.score - avg_score;
			avg_score += delta / k_successes;
			var_score += delta * ( result.score - avg_score );
			avg_error += result.error / ( double )ITER;
		}
	}

	avg_non_success_error = k_successes != ITER ? avg_non_success_error / ( double )( ITER - k_successes ) : 0.0;
	var_score = var_score / ( double )ITER;
	double std_score = std::sqrt( std::max( 0.0, var_score ) );
	EXPECT_LE( std_score, 0.1 );
	EXPECT_LE( avg_score, 0.25 );
	EXPECT_LE( avg_error, 2 * error_tolerance );
	EXPECT_LE( avg_non_success_error, 2 * error_tolerance );
	EXPECT_LE( max_non_success_error, 5 * error_tolerance );
	EXPECT_LE( avg_iterations, 20 );
	EXPECT_GE( k_successes, 0.95 * ITER );

	std::cout << "Avg score   = " << avg_score << std::endl;
	std::cout << "Std score   = " << std_score << std::endl;
	std::cout << "Avg iter    = " << avg_iterations << std::endl;
	std::cout << "Avg error   = " << avg_error << std::endl;
	std::cout << "Avg fail err= " << avg_non_success_error << std::endl;
	std::cout << "Max fail err= " << max_non_success_error << std::endl;
	std::cout << "k_successes = " << k_successes << " / " << ITER << std::endl;
}

// ------------------------------------------------------------
// ------------------------------------------------------------

TEST_F( PipelineSolverTest, InverseKinematic_Unreachable )
{
	// Create an unreachable target pose
	Mat4d target_pose = Mat4d::Identity();
	target_pose.block< 3, 1 >( 0, 3 ) = Vec3d( 100.0, 0.0, 0.0 );  // Very far away

	// Seed joints
	VecXd seed( 5 );
	seed << 0, 0, 0, 0, 0;

	auto problem = CreateProblem( seed, target_pose );
	auto result = solver_->Solve( problem, Solver::IKRunContext() );

	// Check that the solution is not successful
	EXPECT_FALSE( result.Success() ) << "IK should fail for unreachable target";
}

// ------------------------------------------------------------
// ------------------------------------------------------------

TEST_F( PipelineSolverTest, InverseKinematic_WithDifferentSeedJoints )
{
	// Target joints
	VecXd joints( 5 );
	joints << M_PI / 2, M_PI / 2, M_PI / 3, M_PI / 4, M_PI / 5;
	auto target_pose = ComputeFK( model_, joints );

	// Test with different seed joints
	std::vector< Eigen::Vector< double, 5 >> seed_joints_list = {
		{ 0, 0, 0, 0, 0 },
		{ M_PI / 4, M_PI / 4, M_PI / 4, M_PI / 4, M_PI / 4 },
		{ -M_PI / 4, -M_PI / 4, -M_PI / 4, -M_PI / 4, -M_PI / 4 },
	};

	for ( const auto& seed_joints : seed_joints_list )
	{
		auto problem = CreateProblem( seed_joints, target_pose );
		auto result = solver_->Solve( problem, Solver::IKRunContext() );

		// Check that the solution is valid
		EXPECT_TRUE( result.Success() );
	}
}

// ------------------------------------------------------------

} // namespace SOArm100::Kinematics::Test
