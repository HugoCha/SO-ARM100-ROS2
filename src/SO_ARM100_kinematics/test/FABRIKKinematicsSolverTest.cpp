#include "FABRIKSolver/FabrikSolver.hpp"

#include "Global.hpp"

#include "DLSSolver/NumericSolverResult.hpp"
#include "Model/JointChain.hpp"
#include "Model/Limits.hpp"
#include "RobotModelTestData.hpp"
#include "Utils/Converter.hpp"
#include "Utils/KinematicsUtils.hpp"

#include <gtest/gtest.h>
#include <memory>
#include <random_numbers/random_numbers.h>
#include <vector>

namespace SOArm100::Kinematics::Test
{

// ------------------------------------------------------------
// Fixture
// ------------------------------------------------------------

class FABRIKKinematicsSolverTest : public ::testing::Test
{
protected:
void SetUp() override
{
	auto robot_joint_chain = Data::Create5DofRobotJointChain();
	auto base_joint = robot_joint_chain.GetActiveJoint( 0 );
	auto elbow_joint = robot_joint_chain.GetActiveJoint( 2 );
	auto robot_without_wrist = robot_joint_chain.SubChain( base_joint, elbow_joint );
	joint_chain_ = std::make_shared< JointChain >( robot_without_wrist );

	Mat4d home = ToTransformMatrix( Vec3d( 0, 0, 1.5 ) );
	home_configuration_ = std::make_shared< Mat4d >( home );

	solver_ = std::make_unique< FABRIKKinematicsSolver >();
	solver_->Initialize( joint_chain_, home_configuration_, 0.01 );
}

// Random joints within limits with optional margin
VecXd RandomValidJoints( double margin_percent = 0.05 ) const
{
	VecXd q( joint_chain_->GetActiveJointCount() );
	for ( size_t i = 0; i < joint_chain_->GetActiveJointCount(); i++ )
		joint_chain_->GetActiveJointLimits( i ).Random(
			rng_, & q.data()[i], margin_percent );
	return q;
}

VecXd RandomValidJointsNear(
	const VecXd& joints,
	double distance,
	double margin_percent = 0.05 ) const
{
	VecXd q( joint_chain_->GetActiveJointCount() );
	for ( size_t i = 0; i < joint_chain_->GetActiveJointCount(); i++ )
		joint_chain_->GetActiveJointLimits( i ).RandomNear(
			rng_, joints[i], & q.data()[i], distance, margin_percent );
	return q;
}

std::shared_ptr< JointChain >           joint_chain_;
std::shared_ptr< Mat4d >                home_configuration_;
std::unique_ptr< FABRIKKinematicsSolver > solver_;
mutable random_numbers::RandomNumberGenerator rng_;
};

// ------------------------------------------------------------
// Basic convergence with known joint configuration
// ------------------------------------------------------------

TEST_F( FABRIKKinematicsSolverTest, IK_ConvergesFromNearSeed )
{
	VecXd joints( 3 );
	joints << 0.4, 0.3, 0.25;
	std::vector< double > seed {
		joints[0] + 0.03,
		joints[1] + 0.04,
		joints[2] - 0.04
	};

	Mat4d target;
	POE( *joint_chain_, *home_configuration_, joints, target );

	auto result = solver_->InverseKinematic(
		target,
		seed );

	Mat4d result_pose;
	POE( *joint_chain_, *home_configuration_, result.joints, result_pose );

	EXPECT_TRUE( result.Success() )
	    << "State   = " << static_cast< int >( result.state ) << "\n"
	    << "Error   = " << result.final_error << "\n"
	    << "Joints  = " << result.joints.transpose() << "\n"
	    << "Target  =\n" << Translation( target ).transpose() << "\n"
	    << "Result  =\n" << Translation( result_pose ).transpose() << std::endl;

	EXPECT_TRUE( TranslationError( target, result_pose ) < solver_->GetParameters().error_tolerance )
	    << "Joints  = " << result.joints.transpose() << "\n"
	    << "Target  =\n" << Translation( target ).transpose() << "\n"
	    << "Result  =\n" << Translation( result_pose ).transpose() << std::endl;
}

// ------------------------------------------------------------
// Zero-config: seed = solution, should converge in very few iterations
// ------------------------------------------------------------

TEST_F( FABRIKKinematicsSolverTest, IK_ConvergesFromExactSeed )
{
	VecXd joints( 3 );
	joints << 0, M_PI / 4, -M_PI / 4;
	Mat4d target;
	POE( *joint_chain_, *home_configuration_, joints, target );

	std::vector< double > seed_vec( joints.data(), joints.data() + joints.size() );
	auto result = solver_->InverseKinematic( target, seed_vec );

	EXPECT_TRUE( result.Success() )
	    << "Exact seed should converge immediately. "
	    << "Error = " << result.final_error;
}

// ------------------------------------------------------------
// Straight-up configuration (all zeros)
// ------------------------------------------------------------

TEST_F( FABRIKKinematicsSolverTest, IK_ZeroConfiguration )
{
	VecXd joints = VecXd::Zero( 3 );
	Mat4d target;
	POE( *joint_chain_, *home_configuration_, joints, target );

	std::vector< double > seed{ 0.1, 0.1, 0.1 };
	auto result = solver_->InverseKinematic( target, seed );

	EXPECT_TRUE( result.Success() )
	    << "Zero configuration should be reachable. "
	    << "State   = " << static_cast< int >( result.state ) << "\n"
	    << "Error   = " << result.final_error << "\n"
	    << "Joints  = " << result.joints.transpose() << "\n"
	    << "Target  =\n" << Translation( target ).transpose() << "\n";
}

// ------------------------------------------------------------
// Unreachable: target far outside workspace
// ------------------------------------------------------------

TEST_F( FABRIKKinematicsSolverTest, IK_UnreachableFarTarget )
{
	Mat4d target     = Mat4d::Identity();
	target( 0, 3 )   = 100.0;
	target( 1, 3 )   = 0.0;
	target( 2, 3 )   = 0.0;

	std::vector< double > seed{ 0, 0, 0 };
	auto result = solver_->InverseKinematic( target, seed );

	EXPECT_FALSE( result.Success() )
	    << "Target at (100,0,0) is outside workspace, IK should not succeed.";
}

// ------------------------------------------------------------
// Multiple seeds: solution is seed-independent
// ------------------------------------------------------------

TEST_F( FABRIKKinematicsSolverTest, IK_MultipleSeedsConverge )
{
	VecXd joints( 3 );
	joints << M_PI / 4, 0, M_PI / 5;
	Mat4d target;
	POE( *joint_chain_, *home_configuration_, joints, target );


	std::vector< std::vector< double >> seeds = {
		{ 0, 0, 0 },
		{ 0.3, 0.4, -0.2 },
		{ -0.3, -0.4, 0.2 },
		{ M_PI / 4, M_PI / 4, M_PI / 4  },
	};

	for ( size_t s = 0; s < seeds.size(); s++ )
	{
		auto result = solver_->InverseKinematic( target, seeds[s] );
		EXPECT_TRUE( result.Success() )
		    << "Seed index " << s << " failed. "
		    << "Error = " << result.final_error;
	}
}

// ------------------------------------------------------------
// Position accuracy: FK( result ) ≈ target position
// ------------------------------------------------------------

TEST_F( FABRIKKinematicsSolverTest, IK_PositionAccuracy )
{
	VecXd joints( 3 );
	joints << M_PI / 4, 0, M_PI / 4;
	Mat4d target;
	POE( *joint_chain_, *home_configuration_, joints, target );

	std::vector< double > seed{ 0., -0.7, 0.8 };
	auto result = solver_->InverseKinematic( target, seed );

	ASSERT_TRUE( result.Success() ) << "IK must succeed before checking accuracy.";

	Mat4d result_pose;
	POE( *joint_chain_, *home_configuration_, result.joints, result_pose );

	double pos_error  = TranslationError( target, result_pose );

	EXPECT_LT( pos_error, solver_->GetParameters().error_tolerance )
	    << "Position error = " << pos_error << std::endl
	    << "Result joints = " << result.joints.transpose() << std::endl
	    << "Target = " << Translation( target ).transpose() << std::endl
	    << "Result = " << Translation( result_pose ).transpose() << std::endl;
}

// ------------------------------------------------------------
// Joint limits: result joints must stay within limits
// ------------------------------------------------------------

TEST_F( FABRIKKinematicsSolverTest, IK_JointLimitsRespected )
{
	VecXd joints( 3 );
	joints << M_PI / 3, 3 * M_PI / 2, M_PI / 4;
	Mat4d target;
	POE( *joint_chain_, *home_configuration_, joints, target );


	std::vector< double > seed{ 0, 0, 0 };
	auto result = solver_->InverseKinematic( target, seed );

	ASSERT_TRUE( result.Success() );

	for ( size_t i = 0; i < result.joints.size(); i++ )
	{
		const auto& limits = joint_chain_->GetActiveJointLimits( i );
		EXPECT_TRUE( limits.Within( result.joints[i] ) )
		    << "Joint " << i << " = " << result.joints[i]
		    << " violates limits [" << limits.Min() << ", " << limits.Max() << "]";
	}
}

// ------------------------------------------------------------
// Randomised sweep: N random configurations
// ------------------------------------------------------------

TEST_F( FABRIKKinematicsSolverTest, IK_RandomConfigurations )
{
	constexpr int kTrials = 100;
	int successes         = 0;
	double avg_iter = 0.0;
	for ( int t = 0; t < kTrials; t++ )
	{
		VecXd joints = RandomValidJoints( 0.05 );
		Mat4d target;
		POE( *joint_chain_, *home_configuration_, joints, target );


		VecXd seed_joints = RandomValidJointsNear( joints, 0.3, 0.05 );
		std::vector< double > seed( seed_joints.data(),
		                            seed_joints.data() + seed_joints.size() );

		auto result = solver_->InverseKinematic( target, seed );
		if ( result.Success() )
			successes++;
		avg_iter += result.iterations_used / ( double )kTrials;
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

TEST_F( FABRIKKinematicsSolverTest, IK_IterationCountReasonable )
{
	VecXd joints( 3 );
	joints << M_PI / 3, M_PI / 4, M_PI / 5;
	Mat4d target;
	POE( *joint_chain_, *home_configuration_, joints, target );


	std::vector< double > seed{ 0.3, 0.4, 0.5 };
	auto result = solver_->InverseKinematic( target, seed );

	EXPECT_GE( result.iterations_used, 0 );
	EXPECT_LE( result.iterations_used, solver_->GetParameters().max_iterations );
}

// ------------------------------------------------------------
// Custom parameters: tighter tolerance converges to better solution
// ------------------------------------------------------------

TEST_F( FABRIKKinematicsSolverTest, IK_TighterToleranceImproves )
{
	VecXd joints( 3 );
	joints << M_PI / 3, M_PI / 4, M_PI / 5;
	Mat4d target;
	POE( *joint_chain_, *home_configuration_, joints, target );

	std::vector< double > seed{ 0, 0, 0 };

	// Default tolerance
	auto result_default = solver_->InverseKinematic( target, seed );

	// Tighter tolerance, more iterations
	FABRIKKinematicsSolver::SolverParameters tight_params;
	tight_params.max_iterations  = 100;
	tight_params.error_tolerance = 1e-6;
	solver_->SetParameters( tight_params );
	auto result_tight = solver_->InverseKinematic( target, seed );

	// Reset
	solver_->SetParameters( FABRIKKinematicsSolver::SolverParameters{} );

	if ( result_default.Success() && result_tight.Success() )
	{
		EXPECT_LE( result_tight.final_error, result_default.final_error + 1e-9 )
		    << "Tighter tolerance should not produce worse error.";
	}
}

// ------------------------------------------------------------
// Boundary joints: solution near joint limits should still converge
// ------------------------------------------------------------

TEST_F( FABRIKKinematicsSolverTest, IK_NearJointLimits )
{
	// Push joints close to their limits
	VecXd joints( 6 );
	joints << M_PI * 0.9,          // near ±π
	    M_PI / 2 * 0.9,             // near ±π/2
	    M_PI / 2 * 0.9;            // near ±π/2

	Mat4d target;
	POE( *joint_chain_, *home_configuration_, joints, target );


	std::vector< double > seed{ 0, 0, 0 };
	auto result = solver_->InverseKinematic( target, seed );

	Mat4d result_pose;
	POE( *joint_chain_, *home_configuration_, result.joints, result_pose );

	double pos_error  = TranslationError( target, result_pose );

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

TEST_F( FABRIKKinematicsSolverTest, IK_SeedSizeMismatchFails )
{
	Mat4d target = Mat4d::Identity();

	std::vector< double > wrong_seed{ 0, 0, 0, 0 }; // only 3 joints, need 6
	auto result = solver_->InverseKinematic( target, wrong_seed );

	EXPECT_EQ( result.state, NumericSolverState::Failed )
	    << "Mismatched seed size should return Failed state.";
}

// ------------------------------------------------------------

} // namespace SOArm100::Kinematics::Test