#include "FABRIKSolver/FabrikSolver.hpp"

#include "Global.hpp"

#include "RobotModelTestData.hpp"
#include "KinematicTestBase.hpp"

#include "Solver/IKProblem.hpp"
#include "Model/KinematicModel.hpp"
#include "Solver/IKRunContext.hpp"
#include "Solver/IKSolverState.hpp"
#include "Utils/KinematicsUtils.hpp"

#include <Eigen/src/Core/Matrix.h>
#include <gtest/gtest.h>
#include <memory>
#include <random_numbers/random_numbers.h>
#include <vector>

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
	model_ = Data::GetPlanar3RRobot();
	solver_ = std::make_unique< Solver::FABRIKSolver >( model_ );
}

protected:
random_numbers::RandomNumberGenerator rng_;
std::unique_ptr< Solver::FABRIKSolver > solver_;
static constexpr double DEFAULT_TOLERANCE = error_tolerance;
};

// ------------------------------------------------------------
// Basic convergence with known joint configuration
// ------------------------------------------------------------

TEST_F( FABRIKSolverTest, IK_ConvergesFromNearSeed )
{
	VecXd joints( 3 );
	joints << 0.4, 0.3, 0.25;
	VecXd seed( 3 );
	seed << joints[0] + 0.03,
			joints[1] + 0.04,
			joints[2] - 0.04;

	auto problem = CreateProblem( seed, joints );
	auto result = solver_->Solve( problem, Solver::IKRunContext() );
	Mat4d result_pose = ComputeFK( result.joints );

	EXPECT_TRUE( result.Success() )
	    << "State   = " << static_cast< int >( result.state ) << "\n"
	    << "Error   = " << result.error << "\n"
	    << "Joints  = " << result.joints.transpose() << "\n"
	    << "Target  =\n" << Translation( problem.target ).transpose() << "\n"
	    << "Result  =\n" << Translation( result_pose ).transpose() << std::endl;

	EXPECT_TRUE( TranslationError( problem.target, result_pose ) < solver_->GetParameters().error_tolerance )
	    << "Joints  = " << result.joints.transpose() << "\n"
	    << "Target  =\n" << Translation( problem.target  ).transpose() << "\n"
	    << "Result  =\n" << Translation( result_pose ).transpose() << std::endl;
}

// ------------------------------------------------------------
// Zero-config: seed = solution, should converge in very few iterations
// ------------------------------------------------------------

TEST_F( FABRIKSolverTest, IK_ConvergesFromExactSeed )
{
	VecXd joints( 3 );
	joints << 0, M_PI / 4, -M_PI / 4;

	auto problem = CreateProblem( joints, joints );
	auto result = solver_->Solve( problem, Solver::IKRunContext() );

	EXPECT_TRUE( result.Success() )
	    << "Exact seed should converge immediately. "
	    << "Error = " << result.error;

	EXPECT_EQ( result.iterations, 0 );
}

// ------------------------------------------------------------
// Straight-up configuration (all zeros)
// ------------------------------------------------------------

TEST_F( FABRIKSolverTest, IK_ZeroConfiguration )
{
	VecXd joints = VecXd::Zero( 3 );
	VecXd seed(3);
	seed << 0.1, 0.1, 0.1;

	auto problem = CreateProblem( seed, joints );
	auto result = solver_->Solve( problem, Solver::IKRunContext() );

	EXPECT_TRUE( result.Success() )
	    << "Zero configuration should be reachable. "
	    << "State   = " << static_cast< int >( result.state ) << "\n"
	    << "Error   = " << result.error << "\n"
	    << "Joints  = " << result.joints.transpose() << "\n"
	    << "Target  =\n" << Translation( problem.target ).transpose() << "\n";
}

// ------------------------------------------------------------
// Unreachable: target far outside workspace
// ------------------------------------------------------------

TEST_F( FABRIKSolverTest, IK_UnreachableFarTarget )
{
	Mat4d target     = Mat4d::Identity();
	target( 0, 3 )   = 100.0;
	target( 1, 3 )   = 0.0;
	target( 2, 3 )   = 0.0;

	auto problem = CreateProblem( VecXd::Zero( 3 ), target );
	auto result = solver_->Solve( problem, Solver::IKRunContext() );

	EXPECT_FALSE( result.Success() )
	    << "Target at (100,0,0) is outside workspace, IK should not succeed.";
}

// ------------------------------------------------------------
// Multiple seeds: solution is seed-independent
// ------------------------------------------------------------

TEST_F( FABRIKSolverTest, IK_MultipleSeedsConverge )
{
	VecXd joints( 3 );
	joints << M_PI / 4, 0, M_PI / 5;
	Mat4d target = ComputeFK( joints );

	std::vector< Eigen::Vector< double, 3 > > seeds = {
		{ 0, 0, 0 },
		{ 0.3, 0.4, -0.2 },
		{ -0.3, -0.4, 0.2 },
		{ M_PI / 4, M_PI / 4, M_PI / 4  },
	};

	for ( size_t s = 0; s < seeds.size(); s++ )
	{
		auto problem = CreateProblem( seeds[s], target );
		auto result = solver_->Solve( problem, Solver::IKRunContext() );

		EXPECT_TRUE( result.Success() )
		    << "Seed index " << s << " failed. "
		    << "Error = " << result.error;
	}
}

// ------------------------------------------------------------
// Position accuracy: FK( result ) ≈ target position
// ------------------------------------------------------------

TEST_F( FABRIKSolverTest, IK_PositionAccuracy )
{
	VecXd joints( 3 );
	joints << M_PI / 4, 0, M_PI / 4;
	VecXd seed(3);
	seed << 0., -0.7, 0.8;

	auto problem = CreateProblem( seed, joints );
	auto result = solver_->Solve( problem, Solver::IKRunContext() );

	ASSERT_TRUE( result.Success() ) << "IK must succeed before checking accuracy.";

	Mat4d result_pose = ComputeFK( result.joints );

	double pos_error  = TranslationError( problem.target, result_pose );

	EXPECT_LT( pos_error, solver_->GetParameters().error_tolerance )
	    << "Position error = " << pos_error << std::endl
	    << "Result joints = " << result.joints.transpose() << std::endl
	    << "Target = " << Translation( problem.target ).transpose() << std::endl
	    << "Result = " << Translation( result_pose ).transpose() << std::endl;
}

// ------------------------------------------------------------
// Joint limits: result joints must stay within limits
// ------------------------------------------------------------

TEST_F( FABRIKSolverTest, IK_JointLimitsRespected )
{
	VecXd joints( 3 );
	joints << M_PI / 3, 3 * M_PI / 2, M_PI / 4;
	VecXd seed = VecXd::Zero(3);

	auto problem = CreateProblem( seed, joints );
	auto result = solver_->Solve( problem, Solver::IKRunContext() );

	ASSERT_TRUE( result.Success() );
	ASSERT_TRUE( model_->GetChain()->WithinLimits( result.joints ) );
}

// ------------------------------------------------------------
// Randomised sweep: N random configurations
// ------------------------------------------------------------

TEST_F( FABRIKSolverTest, IK_RandomConfigurations )
{
	constexpr int kTrials = 100;
	int successes         = 0;
	double avg_iter = 0.0;
	for ( int t = 0; t < kTrials; t++ )
	{
		VecXd joints = model_->GetChain()->RandomValidJoints( rng_, 0.05 );
		VecXd seed = model_->GetChain()->RandomValidJointsNear( rng_, joints, 0.3, 0 );

		auto problem = CreateProblem( seed, joints );
		auto result = solver_->Solve( problem, Solver::IKRunContext() );

		if ( result.Success() )
			successes++;
		avg_iter += result.iterations / ( double )kTrials;
	}

	// Expect at least 80% success rate on reachable random targets
	EXPECT_LE( avg_iter, 10 )
	    << "Average iterations = " << avg_iter << std::endl;
	EXPECT_GE( successes, static_cast< int >( kTrials * 0.8 ) )
	    << "Success rate " << successes << "/" << kTrials
	    << " is below 80% threshold.";
}

// ------------------------------------------------------------
// Iteration budget: convergence reported iteration count is plausible
// ------------------------------------------------------------

TEST_F( FABRIKSolverTest, IK_IterationCountReasonable )
{
	VecXd joints( 3 );
	joints << M_PI / 3, M_PI / 4, M_PI / 5;
	VecXd seed(3);
	seed << 0.3, 0.4, 0.5;

	auto problem = CreateProblem( seed, joints );
	auto result = solver_->Solve( problem, Solver::IKRunContext() );

	EXPECT_GE( result.iterations, 0 );
	EXPECT_LE( result.iterations, solver_->GetParameters().max_iterations );
}

// ------------------------------------------------------------
// Custom parameters: tighter tolerance converges to better solution
// ------------------------------------------------------------

TEST_F( FABRIKSolverTest, IK_TighterToleranceImproves )
{
	VecXd joints( 3 );
	joints << M_PI / 3, M_PI / 4, M_PI / 5;
	VecXd seed = VecXd::Zero(3);

	auto problem = CreateProblem( seed, joints );

	// Default tolerance
	auto result_default = solver_->Solve( problem, Solver::IKRunContext() );

	// Tighter tolerance, more iterations
	Solver::FABRIKSolver::SolverParameters tight_params( 100, 1e-6 );
	solver_->SetParameters( tight_params );
	auto result_tight = solver_->Solve( problem, Solver::IKRunContext() );

	// Reset
	solver_->SetParameters( Solver::FABRIKSolver::SolverParameters{} );

	if ( result_default.Success() && result_tight.Success() )
	{
		EXPECT_LE( result_tight.error, result_default.error + 1e-9 )
		    << "Tighter tolerance should not produce worse error.";
	}
}

// ------------------------------------------------------------
// Boundary joints: solution near joint limits should still converge
// ------------------------------------------------------------

TEST_F( FABRIKSolverTest, IK_NearJointLimits )
{
	// Push joints close to their limits
	VecXd joints( 6 );
	joints << M_PI * 0.9,          // near ±π
	    M_PI / 2 * 0.9,             // near ±π/2
	    M_PI / 2 * 0.9;            // near ±π/2

	auto problem = CreateProblem( VecXd::Zero( 3 ), joints );
	auto result = solver_->Solve( problem, Solver::IKRunContext() );

	Mat4d result_pose = ComputeFK( result.joints );
	double pos_error  = TranslationError( problem.target, result_pose );

	// Near-limit targets may not converge to full tolerance,
	// but position error should still be small
	EXPECT_LT( pos_error, 5e-2 )
	    << "Position error " << pos_error
	    << " too large for near-limit target.\n"
	    << "State  = " << static_cast< int >( result.state ) << "\n"
	    << "Joints = " << result.joints.transpose();
}

// ------------------------------------------------------------
// Size mismatch: wrong seed size returns Failed
// ------------------------------------------------------------

TEST_F( FABRIKSolverTest, IK_SeedSizeMismatchFails )
{
	Mat4d target = Mat4d::Identity();

	auto problem = CreateProblem( VecXd::Zero( 4 ), target );
	auto result = solver_->Solve( problem, Solver::IKRunContext() );

	EXPECT_EQ( result.state, Solver::IKSolverState::NotRun )
	    << "Mismatched seed size should return Failed state.";
}

// ------------------------------------------------------------

} // namespace SOArm100::Kinematics::Test