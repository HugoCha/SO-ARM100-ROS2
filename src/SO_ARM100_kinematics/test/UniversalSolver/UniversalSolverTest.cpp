#include "UniversalSolver/UniversalSolver.hpp"
#include "UniversalSolver/UniversalModel.hpp"
#include "UniversalSolver/UniversalSolution.hpp"
#include "Global.hpp"
#include "KinematicTestBase.hpp"
#include "Model/Joint/Joint.hpp"
#include "Utils/KinematicsUtils.hpp"

#include <gtest/gtest.h>
#include <memory>
#include <cmath>
#include <vector>

namespace SOArm100::Kinematics::Test
{

// ============================================================
// UniversalSolver Test
// ============================================================

class UniversalSolverTest : public KinematicTestBase
{
protected:
void SetUp() override
{
	// Standard Orthogonal Axes
	joint_x_ = MakeRevoluteJoint( Vec3d::UnitX(), Vec3d::Zero(), -M_PI, M_PI );
	joint_y_ = MakeRevoluteJoint( Vec3d::UnitY(), Vec3d::Zero(), -M_PI, M_PI );
	joint_z_ = MakeRevoluteJoint( Vec3d::UnitZ(), Vec3d::Zero(), -M_PI, M_PI );

	// Tight Limits for Bounds Validation
	joint_x_tight_ = MakeRevoluteJoint( Vec3d::UnitX(), Vec3d::Zero(), -M_PI / 6, M_PI / 6 );
	joint_y_tight_ = MakeRevoluteJoint( Vec3d::UnitY(), Vec3d::Zero(), -M_PI / 6, M_PI / 6 );

	// Non-Orthogonal Arbitrary Axes
	Vec3d non_ortho_ax2 = Vec3d( 1.0, 1.0, 0.0 ).normalized();
	joint_non_ortho_1_ = MakeRevoluteJoint( Vec3d::UnitZ(), Vec3d::Zero(), -M_PI, M_PI );
	joint_non_ortho_2_ = MakeRevoluteJoint( non_ortho_ax2,  Vec3d::Zero(), -M_PI, M_PI );

	// Models
	auto model_xy        = Model::UniversalModel::ComputeModel( joint_x_, joint_y_ );
	auto model_tight     = Model::UniversalModel::ComputeModel( joint_x_tight_, joint_y_tight_ );
	auto model_non_ortho = Model::UniversalModel::ComputeModel( joint_non_ortho_1_, joint_non_ortho_2_ );

	EXPECT_TRUE( model_xy.has_value() );
	EXPECT_TRUE( model_tight.has_value() );
	EXPECT_TRUE( model_non_ortho.has_value() );

	parameters_      = Solver::UniversalSolver::SolverParameters();
	solver_xy_       = std::make_unique< Solver::UniversalSolver >( *model_xy,        parameters_ );
	solver_tight_    = std::make_unique< Solver::UniversalSolver >( *model_tight,     parameters_ );
	solver_non_ortho_ = std::make_unique< Solver::UniversalSolver >( *model_non_ortho, parameters_ );
}

void TearDown() override {
}

// ------------------------------------------------------------

struct ConfigurationCase
{
	Model::JointConstPtr j1;
	Model::JointConstPtr j2;
	Vec3d ax1;
	Vec3d ax2;
	const char* name;
};

// ------------------------------------------------------------

std::vector< ConfigurationCase > AllOrthogonalConfigurations() const
{
	return {
		{ joint_x_, joint_y_, Vec3d::UnitX(), Vec3d::UnitY(), "XY" },
		{ joint_y_, joint_x_, Vec3d::UnitY(), Vec3d::UnitX(), "YX" },
		{ joint_x_, joint_z_, Vec3d::UnitX(), Vec3d::UnitZ(), "XZ" },
		{ joint_z_, joint_x_, Vec3d::UnitZ(), Vec3d::UnitX(), "ZX" },
		{ joint_y_, joint_z_, Vec3d::UnitY(), Vec3d::UnitZ(), "YZ" },
		{ joint_z_, joint_y_, Vec3d::UnitZ(), Vec3d::UnitY(), "ZY" },
	};
}

// ------------------------------------------------------------

Mat3d ComputeFK(
	const Model::JointConstPtr& j1,
	const Model::JointConstPtr& j2,
	const Vec2d& angles ) const
{
	return ( AngleAxis( angles[0], j1->Axis() )
	         * AngleAxis( angles[1], j2->Axis() ) ).toRotationMatrix();
}

// ------------------------------------------------------------

// Checks that the solver recovers a rotation that maps b0 → b1
void ExpectVectorMatch(
	const Model::UniversalModel& model,
	const Vec3d& b0,
	const Vec3d& b1,
	const Solver::UniversalSolution& result,
	double tol = 1e-4 )
{
	Mat3d R = model.Recompose( result.angles );
	Vec3d recovered = R * b0;

	EXPECT_TRUE( recovered.isApprox( b1, tol ) )
	    << "Expected: " << b1.transpose()        << "\n"
	    << "Got:      " << recovered.transpose()  << "\n"
	    << "Angles:   " << result.angles.transpose();
}

// ------------------------------------------------------------

// Checks that the solver recovers the exact rotation matrix
void ExpectRotationMatch(
	const Model::JointConstPtr& j1,
	const Model::JointConstPtr& j2,
	const Mat3d& R_target,
	const Solver::UniversalSolution& result,
	double tol = 1e-4 )
{
	Mat3d R_result = ComputeFK( j1, j2, result.angles );
	EXPECT_TRUE( R_result.isApprox( R_target, tol ) )
	    << "Target:\n"  << R_target  << "\n"
	    << "Result:\n"  << R_result  << "\n"
	    << "Angles: "   << result.angles.transpose();
}

// ------------------------------------------------------------

void CheckConfiguration(
	ConfigurationCase c,
	const Vec3d& b0,
	const Vec3d& b1,
	std::optional< Vec2d > theta_pref = std::nullopt )
{
	auto model = Model::UniversalModel::ComputeModel( c.j1, c.j2 );

	ASSERT_TRUE( model.has_value() )
	    << "Failed ComputeModel for " << c.name;

	auto solver = Solver::UniversalSolver( *model, parameters_ );
	auto result = solver.SolveFromTwoVectors( b0, b1, theta_pref );

	Mat3d R = model->Recompose( result.angles );
	Vec3d recovered = R * b0;

	EXPECT_TRUE( recovered.isApprox( b1, 1e-4 ) )
	    << "Config: " << c.name   << " failed.\n"
	    << "Expected: " << b1.transpose()        << "\n"
	    << "Got:      " << recovered.transpose()  << "\n"
	    << "Angles:   " << result.angles.transpose();

	EXPECT_LE( result.fk_error, 1e-4 );
	EXPECT_TRUE( result.reachable );
}

// ------------------------------------------------------------

void CheckConfigurationForAngles(
	ConfigurationCase c,
	const Vec3d& b0,
	const Vec2d& angles,
	std::optional< Vec2d > theta_pref = std::nullopt )
{
	auto model = Model::UniversalModel::ComputeModel( c.j1, c.j2 );

	ASSERT_TRUE( model.has_value() )
	    << "Failed ComputeModel for " << c.name;

	Mat3d R_angles = ComputeFK( c.j1, c.j2, angles );
	Vec3d b1 = R_angles * b0;

	auto solver = Solver::UniversalSolver( *model, parameters_ );
	auto result = solver.SolveFromTwoVectors( b0, b1, theta_pref );

	Mat3d R = model->Recompose( result.angles );
	Vec3d recovered = R * b0;

	EXPECT_TRUE( recovered.isApprox( b1, 1e-4 ) )
	    << "Config: " << c.name   << " failed.\n"
	    << "Expected: " << b1.transpose()        << "\n"
	    << "Got:      " << recovered.transpose()  << "\n"
	    << "Angles:   " << result.angles.transpose();

	EXPECT_LE( result.fk_error, 1e-4 );
	EXPECT_TRUE( result.reachable );
}

// ------------------------------------------------------------

void CheckAllOrthogonalConfigurations(
	const Vec3d& b0,
	const Vec3d& b1,
	std::optional< Vec2d > theta_pref = std::nullopt )
{
	for ( const auto& c : AllOrthogonalConfigurations() )
		CheckConfiguration( c, b0, b1, theta_pref );
}

// ------------------------------------------------------------

void CheckAllOrthogonalConfigurationsForAngles(
	const Vec3d& b0,
	const Vec2d& angles,
	std::optional< Vec2d > theta_pref = std::nullopt )
{
	for ( const auto& c : AllOrthogonalConfigurations() )
		CheckConfigurationForAngles( c, b0, angles, theta_pref );
}

// ------------------------------------------------------------

Model::JointConstPtr joint_x_, joint_y_, joint_z_;
Model::JointConstPtr joint_x_tight_, joint_y_tight_;
Model::JointConstPtr joint_non_ortho_1_, joint_non_ortho_2_;

std::unique_ptr< Solver::UniversalSolver > solver_xy_;
std::unique_ptr< Solver::UniversalSolver > solver_tight_;
std::unique_ptr< Solver::UniversalSolver > solver_non_ortho_;
Solver::UniversalSolver::SolverParameters parameters_;
};

// ============================================================
// ComputeModel — validity checks
// ============================================================

TEST_F( UniversalSolverTest, ComputeModel_OrthogonalAxes_ReturnsModel )
{
	auto model = Model::UniversalModel::ComputeModel( joint_x_, joint_y_ );
	EXPECT_TRUE( model.has_value() );
}

// ------------------------------------------------------------

TEST_F( UniversalSolverTest, ComputeModel_ParallelAxes_ReturnsNullopt )
{
	// Parallel axes cannot form a valid universal joint
	auto joint_x2 = MakeRevoluteJoint( Vec3d::UnitX(), Vec3d::Zero(), -M_PI, M_PI );
	auto model = Model::UniversalModel::ComputeModel( joint_x_, joint_x2 );
	EXPECT_FALSE( model.has_value() );
}

// ============================================================
// Recompose / FK roundtrip
// ============================================================

TEST_F( UniversalSolverTest, Recompose_KnownAngles_MatchesManualFK )
{
	const auto& model = solver_xy_->GetModel();
	Vec2d angles( M_PI / 4, M_PI / 6 );

	Mat3d R_recompose = model.Recompose( angles );
	Mat3d R_manual    = ComputeFK( joint_x_, joint_y_, angles );

	EXPECT_TRUE( R_recompose.isApprox( R_manual, 1e-10 ) );
}

// ============================================================
// SolveFromTwoVectors — identity (same vector)
// ============================================================

TEST_F( UniversalSolverTest, SolveFromTwoVectors_SameVector_ReturnsNearZeroAngles )
{
	Vec3d b = Vec3d::UnitZ();
	auto result = solver_xy_->SolveFromTwoVectors( b, b );

	EXPECT_TRUE( result.reachable );
	EXPECT_NEAR( result.fk_error, 0.0, 1e-4 );
	EXPECT_TRUE( result.angles.isZero( 1e-4 ) )
	    << "Expected near-zero angles, got: " << result.angles.transpose();
}

// ============================================================
// SolveFromTwoVectors — all orthogonal axis pairs
// ============================================================

TEST_F( UniversalSolverTest, SolveFromTwoVectors_CombinedAngles_AllOrthogonalConfigurations )
{
	Vec3d b0 = Vec3d::UnitX();
	Vec2d angles( M_PI / 4, M_PI / 6 );

	CheckAllOrthogonalConfigurationsForAngles( b0, angles );
}

// ------------------------------------------------------------

TEST_F( UniversalSolverTest, SolveFromTwoVectors_OppositeVector_DirectionMatches )
{
	// 180° — numerically tricky
	Vec3d b0 = Vec3d::UnitX();
	Vec3d b1 = -Vec3d::UnitX();

	CheckAllOrthogonalConfigurations( b0, b1 );
}

// ============================================================
// SolveFromTwoVectors — preferred angles influence
// ============================================================

TEST_F( UniversalSolverTest, SolveFromTwoVectors_SeedInfluence_PrefersSeedCloserSolution )
{
	Vec3d b0 = Vec3d::UnitZ();
	Vec3d b1 = Vec3d( 0, -1, 0 );

	Vec2d seed_zero( 0.0, 0.0 );
	Vec2d seed_near( M_PI / 2, 0.0 );   // close to canonical solution for this target

	auto result_zero = solver_xy_->SolveFromTwoVectors( b0, b1, seed_zero );
	auto result_seed = solver_xy_->SolveFromTwoVectors( b0, b1, seed_near );

	// Both must reach the target
	ExpectVectorMatch( solver_xy_->GetModel(), b0, b1, result_zero );
	ExpectVectorMatch( solver_xy_->GetModel(), b0, b1, result_seed );

	// Seeded result should be closer to seed_near
	double dist_zero = ( result_zero.angles - seed_near ).norm();
	double dist_seed = ( result_seed.angles - seed_near ).norm();
	EXPECT_LE( dist_seed, dist_zero + epsilon );
}

// ============================================================
// SolveFromTwoVectors — joint limits respected
// ============================================================

TEST_F( UniversalSolverTest, SolveFromTwoVectors_TargetWithinLimits_AnglesSatisfyLimits )
{
	// Small rotation well within ±π/6
	Vec3d b0     = Vec3d::UnitZ();
	Vec2d angles = Vec2d( M_PI / 8, M_PI / 8 );
	Mat3d R      = ComputeFK( joint_x_tight_, joint_y_tight_, angles );
	Vec3d b1     = R * b0;

	auto result = solver_tight_->SolveFromTwoVectors( b0, b1 );

	ExpectVectorMatch( solver_tight_->GetModel(), b0, b1, result, 1e-3 );

	const auto& model = solver_tight_->GetModel();
	for ( int i = 0; i < 2; ++i )
	{
		const auto& lim = model.GetJoint( i )->GetLimits();
		EXPECT_GE( result.angles[i], lim.Min() - epsilon ) << "Joint " << i << " below min";
		EXPECT_LE( result.angles[i], lim.Max() + epsilon ) << "Joint " << i << " above max";
	}
}

// ------------------------------------------------------------

TEST_F( UniversalSolverTest, SolveFromTwoVectors_TargetOutsideLimits_ReturnsBestEffort )
{
	// 90° rotation — beyond ±30° tight limits
	Vec3d b0 = Vec3d::UnitZ();
	Vec3d b1 = Vec3d::UnitX();

	auto result = solver_tight_->SolveFromTwoVectors( b0, b1 );

	// Must not crash; angles must stay within limits
	const auto& model = solver_tight_->GetModel();
	for ( int i = 0; i < 2; ++i )
	{
		const auto& lim = model.GetJoint( i )->GetLimits();
		EXPECT_GE( result.angles[i], lim.Min() - epsilon ) << "Joint " << i << " below min";
		EXPECT_LE( result.angles[i], lim.Max() + epsilon ) << "Joint " << i << " above max";
	}
}

// ============================================================
// SolveFromRotation — identity
// ============================================================

TEST_F( UniversalSolverTest, SolveFromRotation_Identity_ReturnsNearZeroAngles )
{
	auto result = solver_xy_->SolveFromRotation( Mat3d::Identity() );

	EXPECT_TRUE( result.reachable );
	EXPECT_NEAR( result.fk_error, 0.0, 1e-4 );
	EXPECT_TRUE( result.angles.isZero( 1e-4 ) )
	    << "Expected near-zero angles, got: " << result.angles.transpose();
}

// ============================================================
// SolveFromRotation — orthogonal setup round-trip
// ============================================================

TEST_F( UniversalSolverTest, SolveFromRotation_OrthogonalConfig_MatchesTargetRotation )
{
	Vec2d expected_angles( M_PI / 4, -M_PI / 6 );
	Mat3d R_target = ComputeFK( joint_x_, joint_y_, expected_angles );

	auto result = solver_xy_->SolveFromRotation( R_target );

	EXPECT_TRUE( result.reachable );
	EXPECT_NEAR( result.fk_error, 0.0, 1e-4 );
	ExpectRotationMatch( joint_x_, joint_y_, R_target, result );
}

// ============================================================
// SolveFromRotation — non-orthogonal setup round-trip
// ============================================================

TEST_F( UniversalSolverTest, SolveFromRotation_NonOrthogonalConfig_MatchesTargetRotation )
{
	Vec2d expected_angles( 0.3, -0.5 );
	Mat3d R_target = ComputeFK( joint_non_ortho_1_, joint_non_ortho_2_, expected_angles );

	auto result = solver_non_ortho_->SolveFromRotation( R_target );

	EXPECT_TRUE( result.reachable );
	EXPECT_NEAR( result.fk_error, 0.0, 1e-4 );
	ExpectRotationMatch( joint_non_ortho_1_, joint_non_ortho_2_, R_target, result );
}

// ============================================================
// SolveFromRotation — outside joint limits
// ============================================================

TEST_F( UniversalSolverTest, SolveFromRotation_OutsideLimits_ReturnsFlaggedBestEffort )
{
	// This angle exceeds ±π/6 tight limits
	Vec2d unreachable( M_PI / 4, M_PI / 4 );
	Mat3d R_target = ComputeFK( joint_x_tight_, joint_y_tight_, unreachable );

	auto result = solver_tight_->SolveFromRotation( R_target );

	EXPECT_FALSE( result.reachable );
	EXPECT_GT( result.fk_error, 1e-4 );

	const auto& model = solver_tight_->GetModel();
	for ( int i = 0; i < 2; ++i )
	{
		const auto& lim = model.GetJoint( i )->GetLimits();
		EXPECT_GE( result.angles[i], lim.Min() - epsilon ) << "Joint " << i << " below min";
		EXPECT_LE( result.angles[i], lim.Max() + epsilon ) << "Joint " << i << " above max";
	}
}

// ============================================================
// SolveFromRotation — consistency
// ============================================================

TEST_F( UniversalSolverTest, SolveFromRotation_SameInput_SameOutput )
{
	Vec2d angles( M_PI / 5, -M_PI / 7 );
	Mat3d R_target = ComputeFK( joint_x_, joint_y_, angles );

	auto r1 = solver_xy_->SolveFromRotation( R_target );
	auto r2 = solver_xy_->SolveFromRotation( R_target );

	EXPECT_TRUE( r1.angles.isApprox( r2.angles, epsilon ) );
}

// ============================================================
// Stress — random angles round-trip (SolveFromTwoVectors)
// ============================================================

TEST_F( UniversalSolverTest, SolveFromTwoVectors_RandomAngles_NonOrthogonal_AlwaysPasses )
{
	random_numbers::RandomNumberGenerator rng;
	const auto& model  = solver_non_ortho_->GetModel();
	const auto& limit0 = model.GetJoint( 0 )->GetLimits();
	const auto& limit1 = model.GetJoint( 1 )->GetLimits();

	Vec3d b0 = Vec3d::UnitX();
	const int ITER = 100;
	int failures = 0;

	for ( int i = 0; i < ITER; ++i )
	{
		Vec2d random_angles;
		limit0.Random( rng, & random_angles[0] );
		limit1.Random( rng, & random_angles[1] );

		Mat3d R  = model.Recompose( random_angles );
		Vec3d b1 = R * b0;

		auto result    = solver_non_ortho_->SolveFromTwoVectors( b0, b1 );
		Mat3d R_result = model.Recompose( result.angles );
		Vec3d recovered = R_result * b0;

		if ( !recovered.isApprox( b1, 1e-3 ) || result.fk_error > 1e-3 )
		{
			++failures;
			ADD_FAILURE()
			    << "Iteration " << i << "\n"
			    << "Seeded angles:  " << random_angles.transpose() << "\n"
			    << "Solved angles:  " << result.angles.transpose() << "\n"
			    << "Expected b1:    " << b1.transpose()            << "\n"
			    << "Recovered:      " << recovered.transpose()     << "\n"
			    << "fk_error:       " << result.fk_error;
		}
	}
	EXPECT_EQ( failures, 0 );
}

// ============================================================
// Stress — random angles round-trip (SolveFromRotation)
// ============================================================

TEST_F( UniversalSolverTest, SolveFromRotation_RandomAngles_NonOrthogonal_AlwaysPasses )
{
	random_numbers::RandomNumberGenerator rng;
	const auto& model  = solver_non_ortho_->GetModel();
	const auto& limit0 = model.GetJoint( 0 )->GetLimits();
	const auto& limit1 = model.GetJoint( 1 )->GetLimits();

	const int ITER = 100;
	int failures = 0;

	for ( int i = 0; i < ITER; ++i )
	{
		Vec2d random_angles;
		limit0.Random( rng, & random_angles[0] );
		limit1.Random( rng, & random_angles[1] );

		Mat3d R_target = model.Recompose( random_angles );
		auto result   = solver_non_ortho_->SolveFromRotation( R_target );
		Mat3d R_result = model.Recompose( result.angles );

		if ( !result.reachable || result.fk_error > 1e-3
		     || RotationError( R_target, R_result ) > 1e-3 )
		{
			++failures;
			ADD_FAILURE()
			    << "Iteration "      << i                             << "\n"
			    << "Seeded angles:   " << random_angles.transpose()   << "\n"
			    << "Solved angles:   " << result.angles.transpose()   << "\n"
			    << "fk_error:        " << result.fk_error             << "\n"
			    << "Rot error:       " << RotationError( R_target, R_result );
		}
	}
	EXPECT_EQ( failures, 0 );
}

// ------------------------------------------------------------

} // namespace SOArm100::Kinematics::Test