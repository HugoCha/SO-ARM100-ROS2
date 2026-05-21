#include "PipelineSolver/PipelineSolver.hpp"

#include "DLS/DLSSolver.hpp"
#include "FABRIK/FabrikSolver.hpp"
#include "PipelineSolver/PipelineBuilder.hpp"
#include "RobotModelTestData.hpp"

#include "Global.hpp"
#include "PipelineSolver/IKPipeline.hpp"
#include "Scorer/CenteredScorer.hpp"
#include "Scorer/CloseToSeedScorer.hpp"
#include "Scorer/IKSolutionScorer.hpp"
#include "Scorer/PoseErrorScorer.hpp"
#include "Scorer/WeightedScorer.hpp"
#include "Scorer/WeightedScorerBuilder.hpp"
#include "Solver/IKRunContext.hpp"
#include "Model/KinematicModel.hpp"
#include "KinematicTestBase.hpp"
#include "Solver/IKSolution.hpp"
#include "Utils/KinematicsUtils.hpp"

#include <Eigen/src/Core/Matrix.h>
#include <gtest/gtest.h>
#include <memory>
#include <vector>

namespace SOArm100::Kinematics::Test
{
// ------------------------------------------------------------
// Pipeline Solver Test
// ------------------------------------------------------------

class IKPipelineTest : public KinematicTestBase
{
protected:
void SetUp() override
{
	model_ = Data::GetRevolute_Planar2R_Wrist2R_5DOFsRobot();

	pipeline_ =
		Solver::PipelineBuilder{}
		.WithHeuristic( std::make_unique< Solver::FABRIKSolver >( model_ ) )
		.WithSolver( std::make_unique< Solver::DLSSolver >( model_ ) )
		.Build();
}

void TearDown() override
{
}

std::unique_ptr< const Solver::IKPipeline > pipeline_;
};

// ------------------------------------------------------------

TEST_F( IKPipelineTest, InverseKinematic_Success )
{
	// Target joints
	VecXd joints( 5 );
	joints << M_PI, M_PI / 2, M_PI / 3, M_PI / 4, M_PI / 5;

	// Seed joints
	VecXd seed( 5 );
	seed << 0, 0, 0, 0, 0;

	auto problem = CreateProblem( model_, seed, joints );
	auto context = Solver::IKRunContext();

	Solver::IKSolution result;
	if ( pipeline_ )
	{
		result = pipeline_->Run( problem, context );
	}

	auto result_pose = ComputeFK( result.joints );

	// Check that the solution is valid
	EXPECT_TRUE( result.Success() ) << "IK should succeed for reachable target";
	EXPECT_EQ( result.joints.size(), model_->GetChain()->GetActiveJointCount() ) << "Result should contain values for all joints";
	EXPECT_TRUE( IsApprox( problem.target, result_pose ) )
	    << "Target=\n" << problem.target << std::endl
	    << "Result=\n" << result_pose << std::endl
	    << "Result joints= " << result.joints.transpose() << std::endl;
}

// ------------------------------------------------------------
// Pipeline Solver Test
// ------------------------------------------------------------

class PipelineSolverTest : public KinematicTestBase
{
protected:
void SetUp() override
{
	model_ = Data::GetRevolute_Planar2R_Wrist2R_5DOFsRobot();

	std::vector< std::unique_ptr< const Solver::IKPipeline >> pipelines;

	pipelines.emplace_back( 
		Solver::PipelineBuilder{}
		.WithHeuristic( std::make_unique< Solver::FABRIKSolver >( model_ ) )
		.WithSolver( std::make_unique< Solver::DLSSolver >( model_ ) )
		.Build() );

	auto scorer = Scorer::WeightedScorerBuilder{}
		.Add( 1.0, std::make_unique< Scorer::CenteredScorer >( model_ ) )
		.Add( 1.0, std::make_unique< Scorer::CloseToSeedScorer >() )
		.Add( 3.0, std::make_unique< Scorer::PoseErrorScorer >( model_ ) )
		.Build();

	// Create a solver
	solver_ = std::unique_ptr< Solver::PipelineSolver >( new Solver::PipelineSolver( 
		model_, 
		std::move( pipelines ), 
		std::move( scorer ),
		Solver::PipelineSolver::SolverParameters() ) );
}

void TearDown() override
{
}

std::unique_ptr< Solver::PipelineSolver > solver_;
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
	seed << 0, 0, 0, 0, 0;

	auto problem = CreateProblem( model_, seed, joints );
	auto result = solver_->Solve( problem, Solver::IKRunContext() );

	auto result_pose = ComputeFK( result.joints );

	// Check that the solution is valid
	EXPECT_TRUE( result.Success() ) << "IK should succeed for reachable target";
	EXPECT_EQ( result.joints.size(), 6 ) << "Result should contain values for all joints";
	EXPECT_TRUE( IsApprox( problem.target, result_pose ) )
	    << "Target=\n" << problem.target << std::endl
	    << "Result=\n" << result_pose << std::endl
	    << "Result joints= " << result.joints.transpose() << std::endl;
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

TEST_F( PipelineSolverTest, InverseKinematic_Singularity )
{
	// Create a target pose that might cause a singularity
	Mat4d target_pose = Mat4d::Identity();
	target_pose.block< 3, 3 >( 0, 0 ) = AngleAxis( M_PI / 2, Vec3d( 0, 1, 0 ) ).toRotationMatrix() *
	                                    AngleAxis( M_PI / 2, Vec3d( 1, 0, 0 ) ).toRotationMatrix();
	target_pose.block< 3, 1 >( 0, 3 ) = Vec3d( 0.5, 0.5, 1.0 );

	// Seed joints
	VecXd seed( 5 );
	seed << 0, 0, 0, 0, 0;

	auto problem = CreateProblem( seed, target_pose );
	auto result = solver_->Solve( problem, Solver::IKRunContext() );

	// Check that the solution is valid (either success or singularity)
	EXPECT_TRUE( result.Success() ) << "IK should succeed or detect singularity for target with potential singularity";
}

// ------------------------------------------------------------

} // namespace SOArm100::Kinematics::Test
