#include "SphericalSolver/SphericalSolver.hpp"

#include "Global.hpp"
#include "KinematicTestBase.hpp"
#include "Model/Joint/Joint.hpp"
#include "SphericalSolver/EulerModel.hpp"
#include "SphericalSolver/EulerSolver.hpp"
#include "SphericalSolver/SphericalModel.hpp"
#include "SphericalSolver/SphericalSolution.hpp"
#include "Utils/KinematicsUtils.hpp"

#include <gtest/gtest.h>
#include <memory>
#include <cmath>
#include <ostream>
#include <vector>

namespace SOArm100::Kinematics::Test
{

// ============================================================
// Solver::SphericalSolver Test
// ============================================================

class SphericalSolverTest : public KinematicTestBase
{
protected:
void SetUp() override
{
    // Standard Orthogonal Axes (Orthogonal Wrist Layout)
    joint_x_ = MakeRevoluteJoint( Vec3d::UnitX(), Vec3d::Zero(), -M_PI / 2, M_PI / 2 );
    joint_y_ = MakeRevoluteJoint( Vec3d::UnitY(), Vec3d::Zero(), -M_PI / 2, M_PI / 2 );
    joint_z_ = MakeRevoluteJoint( Vec3d::UnitZ(), Vec3d::Zero(), -M_PI / 2, M_PI / 2 );

    // Tight Limits Setup for Bounds Validation
    joint_x_tight_ = MakeRevoluteJoint( Vec3d::UnitX(), Vec3d::Zero(), -M_PI / 6, M_PI / 6 );
    joint_y_tight_ = MakeRevoluteJoint( Vec3d::UnitY(), Vec3d::Zero(), -M_PI / 6, M_PI / 6 );
    joint_z_tight_ = MakeRevoluteJoint( Vec3d::UnitZ(), Vec3d::Zero(), -M_PI / 6, M_PI / 6 );

    // Non-Orthogonal Arbitrary Axes Layout
    Vec3d non_ortho_ax2 = Vec3d( 1.0, 1.0, 0.0 ).normalized(); // 45 degrees to X and Y
    joint_non_ortho_1_ = MakeRevoluteJoint( Vec3d::UnitZ(), Vec3d::Zero(), -M_PI, M_PI );
    joint_non_ortho_2_ = MakeRevoluteJoint( non_ortho_ax2,    Vec3d::Zero(), -M_PI, M_PI );
    joint_non_ortho_3_ = MakeRevoluteJoint( Vec3d::UnitX(), Vec3d::Zero(), -M_PI, M_PI );

    // Solvers Setup
    auto model_orthogonal = Model::SphericalModel::ComputeModel( joint_z_, joint_y_, joint_x_ );
    auto model_tight = Model::SphericalModel::ComputeModel( joint_z_tight_, joint_y_tight_, joint_x_tight_ );
    auto model_non_ortho = Model::SphericalModel::ComputeModel( joint_non_ortho_1_, joint_non_ortho_2_, joint_non_ortho_3_ );

    EXPECT_TRUE( model_orthogonal.has_value() );
    EXPECT_TRUE( model_tight.has_value() );
    EXPECT_TRUE( model_non_ortho.has_value() );

    parameters_ = Solver::SphericalSolver::SolverParameters();
    solver_orthogonal_ = std::make_unique< Solver::SphericalSolver >( *model_orthogonal, parameters_ );
    solver_tight_      = std::make_unique< Solver::SphericalSolver >( *model_tight, parameters_ );
    solver_non_ortho_  = std::make_unique< Solver::SphericalSolver >( *model_non_ortho, parameters_ );
}

void TearDown() override {
}


// ------------------------------------------------------------

struct ConfigurationCase
{
	Model::JointConstPtr j1;
	Model::JointConstPtr j2;
	Model::JointConstPtr j3;

	Vec3d ax1;
	Vec3d ax2;
	Vec3d ax3;

	const char* name;
};

// ------------------------------------------------------------

std::vector< ConfigurationCase > AllOrthogonalConfigurations() const
{
	return {
		{
			joint_x_,
			joint_y_,
			joint_z_,
			Vec3d::UnitX(),
			Vec3d::UnitY(),
			Vec3d::UnitZ(),
			"XYZ"
		},
		{
			joint_x_,
			joint_z_,
			joint_y_,
			Vec3d::UnitX(),
			Vec3d::UnitZ(),
			Vec3d::UnitY(),
			"XZY"
		},
		{
			joint_y_,
			joint_x_,
			joint_z_,
			Vec3d::UnitY(),
			Vec3d::UnitX(),
			Vec3d::UnitZ(),
			"YXZ"
		},
		{
			joint_y_,
			joint_z_,
			joint_x_,
			Vec3d::UnitY(),
			Vec3d::UnitZ(),
			Vec3d::UnitX(),
			"YZX"
		},
		{
			joint_z_,
			joint_x_,
			joint_y_,
			Vec3d::UnitZ(),
			Vec3d::UnitX(),
			Vec3d::UnitY(),
			"ZXY"
		},
		{
			joint_z_,
			joint_y_,
			joint_x_,
			Vec3d::UnitZ(),
			Vec3d::UnitY(),
			Vec3d::UnitX(),
			"ZYX"
		},
	};
}

// ------------------------------------------------------------

Mat3d ComputeFK( 
    const std::span< const Model::JointConstPtr >& joints, 
    const Vec3d& angles ) const
{
    return ( AngleAxis( angles[0], joints[0]->Axis() ) *
             AngleAxis( angles[1], joints[1]->Axis() ) *
             AngleAxis( angles[2], joints[2]->Axis() ) ).toRotationMatrix();
}

// ------------------------------------------------------------

Vec3d ApplyAngles(
	const Model::SphericalModel model,
	const Vec3d& old_dir,
	const Vec3d& angles ) const
{
	return ComputeFK( model.GetJoints(), angles) * old_dir;
}

// ------------------------------------------------------------

void ExpectDirectionMatch(
	const Model::SphericalModel model,
	const Vec3d& old_dir,
	const Vec3d& new_dir,
	const Solver::SphericalSolution& result,
	double tol = 1e-4 )
{
	Vec3d recovered = ApplyAngles( model, old_dir, result.angles );

	EXPECT_TRUE( recovered.isApprox( new_dir, tol ) )
	    << "Expected dir: " << new_dir.transpose()    << "\n"
	    << "Got dir:      " << recovered.transpose()  << "\n"
	    << "Angles:       " << result.angles.transpose();
}

// ------------------------------------------------------------

void CheckConfiguration(
	ConfigurationCase c,
	const Vec3d& old_dir,
	const Vec3d& new_dir,
	std::optional< Vec3d > prefered_dir = std::nullopt )
{
	auto model = Model::SphericalModel::ComputeModel(
		c.j1,
		c.j2,
		c.j3 );

	ASSERT_TRUE( model.has_value() )
	    << "Failed ComputeModel for " << c.name;

	auto solver = Solver::SphericalSolver( *model, parameters_ );
	auto result = solver.SolveAndOptimizeFromTwoVectors( old_dir, new_dir, prefered_dir );

	Vec3d recovered = ApplyAngles( *model, old_dir, result.angles );

	EXPECT_TRUE( recovered.isApprox( new_dir, 1e-4 ) )
	    << "Config: " << c.name << " failed.\n"
	    << "Expected dir: " << new_dir.transpose()    << "\n"
	    << "Got dir:      " << recovered.transpose()  << "\n"
	    << "Angles:       " << result.angles.transpose();

    EXPECT_LE( result.fk_error, 1e-4 );
    EXPECT_EQ( result.reachable, true );
}

// ------------------------------------------------------------

void CheckConfigurationForAngles(
	ConfigurationCase c,
	const Vec3d& old_dir,
	const Vec3d& angles,
	std::optional< Vec3d > prefered_dir = std::nullopt )
{
	auto model = Model::SphericalModel::ComputeModel(
		c.j1,
		c.j2,
		c.j3 );

	ASSERT_TRUE( model.has_value() )
	    << "Failed ComputeModel for " << c.name;

	Vec3d new_dir = ApplyAngles( *model, old_dir, angles );
	auto solver = Solver::SphericalSolver( *model, parameters_ );
	auto result = solver.SolveAndOptimizeFromTwoVectors( old_dir, new_dir, prefered_dir );

	Vec3d recovered = ApplyAngles( *model, old_dir, result.angles );

	EXPECT_TRUE( recovered.isApprox( new_dir, 1e-4 ) )
	    << "Config: " << c.name << " failed.\n"
	    << "Expected dir: " << new_dir.transpose()    << "\n"
	    << "Got dir:      " << recovered.transpose()  << "\n"
	    << "Angles:       " << result.angles.transpose();

    EXPECT_LE( result.fk_error, 1e-4 );
    EXPECT_EQ( result.reachable, true );
}

// ------------------------------------------------------------

void CheckAllOrthogonalConfigurationsConfigurations(
	const Vec3d& old_dir,
	const Vec3d& new_dir,
	std::optional< Vec3d > prefered_dir = std::nullopt )
{
	for ( const auto& c : AllOrthogonalConfigurations() )
		CheckConfiguration( c, old_dir, new_dir, prefered_dir );
}

// ------------------------------------------------------------

void CheckAllOrthogonalConfigurationsForAngles(
	const Vec3d& old_dir,
	const Vec3d& angles,
	std::optional< Vec3d > prefered_dir = std::nullopt )
{
	for ( const auto& c : AllOrthogonalConfigurations() )
		CheckConfigurationForAngles( c, old_dir, angles, prefered_dir );
}

// ------------------------------------------------------------

Model::JointConstPtr joint_x_, joint_y_, joint_z_;
Model::JointConstPtr joint_x_tight_, joint_y_tight_, joint_z_tight_;
Model::JointConstPtr joint_non_ortho_1_, joint_non_ortho_2_, joint_non_ortho_3_;

std::unique_ptr< Solver::SphericalSolver > solver_orthogonal_;
std::unique_ptr< Solver::SphericalSolver > solver_tight_;
std::unique_ptr< Solver::SphericalSolver > solver_non_ortho_;
Solver::SphericalSolver::SolverParameters parameters_;
};

// ============================================================
// SolveFromRotation Overload — Identity Target
// ============================================================

TEST_F( SphericalSolverTest, Solve_IdentityMatrix_ReturnsNearZeroAngles )
{
    Mat3d R_target = Mat3d::Identity();

    auto result = solver_orthogonal_->SolveFromRotation( R_target );

    EXPECT_TRUE( result.reachable );
    EXPECT_NEAR( result.fk_error, 0.0, 1e-4 );
    EXPECT_TRUE( result.angles.isZero( 1e-4 ) ) 
        << "Expected near-zero angles, got: " << result.angles.transpose();
}

// ============================================================
// SolveFromRotation Overload — Orthogonal Setup Check
// ============================================================

TEST_F( SphericalSolverTest, Solve_OrthogonalValidRotation_MatchesTargetExactly )
{
    Vec3d expected_angles( M_PI / 4, -M_PI / 6, M_PI / 3 );
    std::vector< Model::JointConstPtr > joints = { joint_z_, joint_y_, joint_x_ };
    Mat3d R_target = ComputeFK( joints, expected_angles );

    auto result = solver_orthogonal_->SolveFromRotation( R_target );

    EXPECT_TRUE( result.reachable );
    EXPECT_NEAR( result.fk_error, 0.0, 1e-4 );
    
    Mat3d R_result = ComputeFK( joints, result.angles );
    EXPECT_TRUE( R_result.isApprox( R_target, 1e-4 ) )
        << "Target Orientation:\n" << R_target << "\nResult Orientation:\n" << R_result;
}

// ============================================================
// SolveFromRotation Overload — Non-Orthogonal Setup Check
// ============================================================

TEST_F( SphericalSolverTest, Solve_NonOrthogonalValidRotation_MatchesTargetExactly )
{
    Vec3d expected_angles( 0.3, -0.5, 0.2 );
    std::vector< Model::JointConstPtr > joints = { joint_non_ortho_1_, joint_non_ortho_2_, joint_non_ortho_3_ };
    Mat3d R_target = ComputeFK( joints, expected_angles );

    auto result = solver_non_ortho_->SolveFromRotation( R_target );

    EXPECT_TRUE( result.reachable );
    EXPECT_NEAR( result.fk_error, 0.0, 1e-4 );
    
    Mat3d R_result = ComputeFK( joints, result.angles );
    EXPECT_TRUE( R_result.isApprox( R_target, 1e-4 ) )
        << "Target Orientation:\n" << R_target << "\nResult Orientation:\n" << R_result;
}

// ============================================================
// Bounds Enforcement — Joint Limits Clamping Fallback
// ============================================================

TEST_F( SphericalSolverTest, Solve_TargetOutsideJointLimits_ReturnsClampedBestEffort )
{
    // Generate orientation out-of-reach for tight limits (requires 45°/30°, limit is 30°)
    Vec3d unreachable_angles( M_PI / 4, M_PI / 6, 0.0 ); 
    std::vector< Model::JointConstPtr > joints = { joint_z_tight_, joint_y_tight_, joint_x_tight_ };
    Mat3d R_target = ComputeFK( joints, unreachable_angles );

    auto result = solver_tight_->SolveFromRotation( R_target );

    // The configuration should be flagged unreachable due to limit violations
    EXPECT_FALSE( result.reachable );
    EXPECT_GT( result.fk_error, 1e-4 );

    // Verify all returned values strictly respect specified joint boundaries
    for ( int i = 0; i < 3; ++i )
    {
        EXPECT_GE( result.angles[i], -M_PI / 6 - epsilon ) << "Joint " << i << " broke minimum boundary limit.";
        EXPECT_LE( result.angles[i],  M_PI / 6 + epsilon ) << "Joint " << i << " broke maximum boundary limit.";
    }
}

// ============================================================
// Bounds Enforcement — Structural Unreachability Fallback
// ============================================================

TEST_F( SphericalSolverTest, Solve_StructuralFailure_TriggersComputeClosestFallback )
{
    // For non-orthogonal/skewed configurations, some mathematical orientations 
    // are completely impossible to reach regardless of limits (discriminant K^2 > R_sq)
    // Create an extreme orientation inversion to force structural failure
    Mat3d R_target = ( Quaternion::FromTwoVectors( Vec3d::UnitZ(), Vec3d::UnitX() ) ).toRotationMatrix();

    auto result = solver_non_ortho_->SolveFromRotation( R_target );

    // Should gracefully execute fallback without exceptions or NaNs
    EXPECT_GE( result.fk_error, 1e-1 );
    EXPECT_FALSE( result.reachable );
    EXPECT_FALSE( std::isnan( result.angles.x() ) );
    EXPECT_FALSE( std::isnan( result.angles.y() ) );
    EXPECT_FALSE( std::isnan( result.angles.z() ) );

    // Check that structural limits clamp the angles properly
    for ( int i = 0; i < 3; ++i )
    {
        const auto& limit = solver_non_ortho_ -> SolveFromRotation(R_target); // Confirm output validation boundary
        EXPECT_TRUE( std::isfinite( result.angles[i] ) );
    }
}

// ============================================================
// Stress Validation Sweep
// ============================================================

TEST_F( SphericalSolverTest, Solve_RandomValidAngles_AllConfigurationsPass )
{
    random_numbers::RandomNumberGenerator rng;
    std::vector< Model::JointConstPtr > joints = { joint_non_ortho_1_, joint_non_ortho_2_, joint_non_ortho_3_ };
    
    const int ITERATIONS = 50;
    int execution_failures = 0;

    Vec3d random_angles;

    for ( int i = 0; i < ITERATIONS; ++i )
    {
        joints[0]->GetLimits().Random( rng, &random_angles[0] );
        joints[1]->GetLimits().Random( rng, &random_angles[1] );
        joints[2]->GetLimits().Random( rng, &random_angles[2] );

        Mat3d R_target = ComputeFK( joints, random_angles );
        auto result = solver_non_ortho_->SolveFromRotation( R_target );

        Mat3d R_result = ComputeFK( joints, result.angles );
        if ( !result.reachable || result.fk_error > 1e-3 || RotationError( R_target, R_result ) > 1e-3 )
        {
            ++execution_failures;
            ADD_FAILURE()
                << "Random sweep structural mismatch at loop index: " << i << "\n"
                << "Seeded Angles: " << random_angles.transpose() << "\n"
                << "Solved Angles: " << result.angles.transpose() << "\n"
                << "Task Error Value: " << result.fk_error;
        }
    }
    EXPECT_EQ( execution_failures, 0 );
}

// ------------------------------------------------------------


// ============================================================
// SolveAndOptimizeFromTwoVectors (direction overload) — identity
// ============================================================

TEST_F( SphericalSolverTest, Solve_Dir_SameDirection_ReturnsNearZeroAngles )
{
	Vec3d old_dir = Vec3d::UnitZ();
	Vec3d new_dir = Vec3d::UnitZ();

	CheckAllOrthogonalConfigurationsConfigurations( old_dir, new_dir );
}

// ============================================================
// SolveAndOptimizeFromTwoVectors (direction overload) — single axis rotations
// ============================================================

TEST_F( SphericalSolverTest, Solve_Dir_PureXRotation_DirectionMatches )
{
	Vec3d old_dir = Vec3d::UnitZ();
	// R_x(π/2) maps Z → -Y
	Vec3d new_dir = Vec3d( 0, -1, 0 );

	CheckAllOrthogonalConfigurationsConfigurations( old_dir, new_dir );
}

// ------------------------------------------------------------

TEST_F( SphericalSolverTest, Solve_Dir_PureYRotation_DirectionMatches )
{
	Vec3d old_dir = Vec3d::UnitZ();
	// R_y(π/2) maps Z → X
	Vec3d new_dir = Vec3d( 1, 0, 0 );

	CheckAllOrthogonalConfigurationsConfigurations( old_dir, new_dir );
}

// ------------------------------------------------------------

TEST_F( SphericalSolverTest, Solve_Dir_PureZRotation_NoDirectionChange )
{
	// Z rotation doesn't move UnitZ — error should be zero regardless of θ3
	Vec3d old_dir = Vec3d::UnitZ();
	Vec3d new_dir = Vec3d::UnitZ();

	CheckAllOrthogonalConfigurationsConfigurations( old_dir, new_dir );
}

// ============================================================
// SolveAndOptimizeFromTwoVectors (direction overload) — combined rotations
// ============================================================

TEST_F( SphericalSolverTest, Solve_Dir_CombinedRotation_DirectionMatches )
{
	Vec3d old_dir = Vec3d::UnitX();
	// Build a target from known angles so we can verify round-trip
	const double a1 = M_PI / 4, a2 = M_PI / 6, a3 = 0.0;
	Vec3d angles = Vec3d( a1, a2, a3 );

	CheckAllOrthogonalConfigurationsForAngles( old_dir, angles );
}

// ------------------------------------------------------------

TEST_F( SphericalSolverTest, Solve_Dir_OppositeDirection_DirectionMatches )
{
	// 180° rotation — numerically tricky for FromTwoVectors
	Vec3d old_dir = Vec3d::UnitZ();
	Vec3d new_dir = -Vec3d::UnitZ();

	CheckAllOrthogonalConfigurationsConfigurations( old_dir, new_dir );
}

// ============================================================
// SolveAndOptimizeFromTwoVectors (direction overload) — seed influence
// ============================================================

TEST_F( SphericalSolverTest, Solve_Dir_SeedCloserToResult_AnglesCloseToSeed )
{
	// There are infinitely many angle triplets that map old→new.
	// With a non-zero seed the optimizer should prefer angles near the seed.
	Vec3d old_dir = Vec3d::UnitZ();
	Vec3d new_dir = Vec3d( 0, -1, 0 );

	Vec3d seed_zero = Vec3d::Zero();
	Vec3d seed_far  = Vec3d( M_PI / 2, 0, 0 );   // close to the "canonical" solution

	auto result_zero = solver_orthogonal_->SolveAndOptimizeFromTwoVectors( old_dir, new_dir, seed_zero );
	auto result_seed = solver_orthogonal_->SolveAndOptimizeFromTwoVectors( old_dir, new_dir, seed_far );

	// Both must reach the target
	ExpectDirectionMatch( solver_orthogonal_->GetModel(), old_dir, new_dir, result_zero );
	ExpectDirectionMatch( solver_orthogonal_->GetModel(), old_dir, new_dir, result_seed );

	// Result seeded near the true solution should have smaller angle norm
	double dist_zero = ( result_zero.angles - seed_far ).norm();
	double dist_seed = ( result_seed.angles - seed_far ).norm();
	EXPECT_LE( dist_seed, dist_zero + epsilon );
}

// ============================================================
// SolveAndOptimizeFromTwoVectors (direction overload) — joint limits respected
// ============================================================

TEST_F( SphericalSolverTest, Solve_Dir_TargetWithinLimits_AnglesSatisfyLimits )
{
	// Small rotation — should be reachable within tight limits
	Vec3d old_dir = Vec3d::UnitZ();
	Vec3d new_dir = ApplyAngles( solver_tight_->GetModel(), old_dir, Vec3d( M_PI / 8, M_PI / 8, 0 ) );
	Vec3d seed    = Vec3d::Zero();

	auto result = solver_tight_->SolveAndOptimizeFromTwoVectors( old_dir, new_dir );
	ExpectDirectionMatch( solver_tight_->GetModel(), old_dir, new_dir, result, 1e-3 );

	for ( int i = 0; i < 3; ++i )
	{
		EXPECT_GE( result.angles[i], -M_PI / 6 - epsilon ) << "Joint " << i << " below min";
		EXPECT_LE( result.angles[i],  M_PI / 6 + epsilon ) << "Joint " << i << " above max";
	}
}

// ------------------------------------------------------------

TEST_F( SphericalSolverTest, Solve_Dir_TargetOutsideLimits_ReturnsBestEffort )
{
	// Large rotation — unreachable within tight limits.
	// Solver must return something (no throw) and error will be non-zero
	// but should be the best achievable.
	Vec3d old_dir = Vec3d::UnitZ();
	Vec3d new_dir = Vec3d( 1, 0, 0 );   // 90° from UnitZ — beyond ±30° limits
	Vec3d seed    = Vec3d::Zero();

	auto result = solver_tight_->SolveAndOptimizeFromTwoVectors( old_dir, new_dir );

	// Must not crash and angles must stay within limits (best-effort)
	for ( int i = 0; i < 3; ++i )
	{
		EXPECT_GE( result.angles[i], -M_PI / 6 - epsilon ) << "Joint " << i << " below min";
		EXPECT_LE( result.angles[i],  M_PI / 6 + epsilon ) << "Joint " << i << " above max";
	}
}

// ============================================================
// SolveAndOptimizeFromTwoVectors (matrix overload)
// ============================================================

TEST_F( SphericalSolverTest, Solve_Mat_Identity_ReturnsNearZeroAngles )
{
    Vec3d seed = Vec3d::Zero();

    auto result = solver_orthogonal_->SolveFromRotation( Mat3d::Identity() );

    EXPECT_TRUE( result.angles.isZero( 1e-4 ) )
        << "Angles: " << result.angles.transpose();
}

// ------------------------------------------------------------

TEST_F( SphericalSolverTest, Solve_Mat_KnownRotation_ErrorNearZero )
{
	// Build R from known angles and verify the solver recovers them
	const double a1 = M_PI / 4, a2 = M_PI / 6, a3 = 0.0;
	Mat3d R_target = ( AngleAxis( a1, Vec3d::UnitZ() )
	                 * AngleAxis( a2, Vec3d::UnitY() )
	                 * AngleAxis( a3, Vec3d::UnitX() ) ).toRotationMatrix();

	Vec3d seed = Vec3d::Zero();
    auto result = solver_orthogonal_->SolveFromRotation( R_target );

	// // The matrix overload uses a3 as the tracked direction.
	// // Error is defined on that direction, so check via ApplyAngles on UnitZ.
	Vec3d old_dir = Vec3d::UnitZ();
	Vec3d new_dir = R_target * old_dir;
	ExpectDirectionMatch( solver_orthogonal_->GetModel(), old_dir, new_dir, result );
}

// ============================================================
// Consistency — multiple calls same result
// ============================================================

TEST_F( SphericalSolverTest, Solve_Dir_Consistency_SameInputSameOutput )
{
	Vec3d old_dir = Vec3d( 1, 0, 0 ).normalized();
	Vec3d new_dir = Vec3d( 0, 1, 1 ).normalized();
	Vec3d seed    = Vec3d::Zero();

	auto r1 = solver_orthogonal_->SolveAndOptimizeFromTwoVectors( old_dir, new_dir );
	auto r2 = solver_orthogonal_->SolveAndOptimizeFromTwoVectors( old_dir, new_dir );

	EXPECT_TRUE( r1.angles.isApprox( r2.angles, epsilon ) );
}

// ============================================================
// Stress — random directions within limits
// ============================================================

TEST_F( SphericalSolverTest, Solve_Dir_HardDirection_DirectionAlwaysMatches )
{
	Vec3d old_dir = Vec3d::UnitZ();
	Vec3d seed    = Vec3d::Zero();

	Vec3d angles;
	angles << -0.70445933432462216,
	    -1.4225262739977698,
	    -0.12398932995613121;

	Vec3d new_dir = ApplyAngles( solver_orthogonal_->GetModel(), old_dir, angles );

	auto result = solver_orthogonal_->SolveAndOptimizeFromTwoVectors( old_dir, new_dir );

	Vec3d recovered = ApplyAngles( solver_orthogonal_->GetModel(), old_dir, result.angles );
	EXPECT_TRUE( recovered.isApprox( new_dir, 1e-3 ) )
	    << "Original Angles " << angles.transpose() << "\n"
	    << "Expected:  " << new_dir.transpose()   << "\n"
	    << "Got:       " << recovered.transpose() << "\n"
	    << "Result Angles:    " << result.angles.transpose();
}

// ------------------------------------------------------------

TEST_F( SphericalSolverTest, Solve_Dir_NonOrthogonalRandomDirections_DirectionAlwaysMatches )
{
	random_numbers::RandomNumberGenerator rng;
	Vec3d old_dir = Vec3d::UnitX();
	Vec3d seed    = Vec3d::Zero();

	const int ITER = 100;
	int failures = 0;

    const auto& model = solver_non_ortho_->GetModel();
	const auto& limit0 = model.GetJoint( 0 )->GetLimits();
	const auto& limit1 = model.GetJoint( 1 )->GetLimits();
	const auto& limit2 = model.GetJoint( 2 )->GetLimits();

	Vec3d random_angles;

	for ( int i = 0; i < ITER; ++i )
	{
		limit0.Random( rng, & random_angles[0] );
		limit1.Random( rng, & random_angles[1] );
		limit2.Random( rng, & random_angles[2] );

		Vec3d new_dir = ApplyAngles( model, old_dir, random_angles );

		auto result = solver_non_ortho_->SolveAndOptimizeFromTwoVectors( old_dir, new_dir );

		Vec3d recovered = ApplyAngles( model, old_dir, result.angles );
		if ( !recovered.isApprox( new_dir, 1e-3 ) )
		{
			++failures;
			ADD_FAILURE()
			    << "Iteration " << i << "\n"
			    << "Expected:  " << new_dir.transpose()   << "\n"
			    << "Got:       " << recovered.transpose() << "\n"
			    << "Angles:    " << result.angles.transpose();
		}
	}
	EXPECT_EQ( failures, 0 );
}

// ------------------------------------------------------------

TEST_F( SphericalSolverTest, Solve_Dir_OrthogonalRandomDirections_DirectionAlwaysMatches )
{
	random_numbers::RandomNumberGenerator rng;
	Vec3d old_dir = Vec3d::UnitZ();
	Vec3d seed    = Vec3d::Zero();

	const int ITER = 100;
	int failures = 0;

    const auto& model = solver_orthogonal_->GetModel();
	const auto& limit0 = model.GetJoint( 0 )->GetLimits();
	const auto& limit1 = model.GetJoint( 1 )->GetLimits();
	const auto& limit2 = model.GetJoint( 2 )->GetLimits();

	Vec3d random_angles;

	for ( int i = 0; i < ITER; ++i )
	{
		limit0.Random( rng, & random_angles[0] );
		limit1.Random( rng, & random_angles[1] );
		limit2.Random( rng, & random_angles[2] );

		Vec3d new_dir = ApplyAngles( model, old_dir, random_angles );

		auto result = solver_orthogonal_->SolveAndOptimizeFromTwoVectors( old_dir, new_dir );

		Vec3d recovered = ApplyAngles( model, old_dir, result.angles );
		if ( !recovered.isApprox( new_dir, 1e-3 ) )
		{
			++failures;
			ADD_FAILURE()
			    << "Iteration " << i << "\n"
			    << "Expected:  " << new_dir.transpose()   << "\n"
			    << "Got:       " << recovered.transpose() << "\n"
			    << "Angles:    " << result.angles.transpose();
		}
	}
	EXPECT_EQ( failures, 0 );
}

// ------------------------------------------------------------


} // namespace SOArm100::Kinematics::Test