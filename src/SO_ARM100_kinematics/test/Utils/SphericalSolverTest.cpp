#include "Euler/SphericalSolver.hpp"
#include "Euler/EulerModel.hpp"

#include "Global.hpp"
#include "KinematicTestBase.hpp"
#include "Model/Joint/Joint.hpp"

#include <gtest/gtest.h>
#include <memory>
#include <cmath>
#include <optional>

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
    joint_x_ = MakeRevoluteJoint(
        Vec3d( 1, 0, 0 ),
        Vec3d( 0, 0, 0 ),
        -M_PI, M_PI );
    joint_y_ = MakeRevoluteJoint(
        Vec3d( 0, 1, 0 ),
        Vec3d( 0, 0, 0 ),
        -M_PI / 2, M_PI / 2 );
    joint_z_ = MakeRevoluteJoint(
        Vec3d( 0, 0, 1 ),
        Vec3d( 0, 0, 0 ),
        -M_PI / 2, M_PI / 2 );

    // Tight limits — used to test limit-violation fallback
    joint_x_tight_ = MakeRevoluteJoint(
        Vec3d( 1, 0, 0 ),
        Vec3d( 0, 0, 0 ),
        -M_PI / 6, M_PI / 6 );
    joint_y_tight_ = MakeRevoluteJoint(
        Vec3d( 0, 1, 0 ),
        Vec3d( 0, 0, 0 ),
        -M_PI / 6, M_PI / 6 );
    joint_z_tight_ = MakeRevoluteJoint(
        Vec3d( 0, 0, 1 ),
        Vec3d( 0, 0, 0 ),
        -M_PI / 6, M_PI / 6 );

    auto model_opt = Model::EulerModel::ComputeModel( 
        joint_z_, 
        joint_y_, 
        joint_x_ );
    ASSERT_TRUE( model_opt.has_value() );
    model_ = std::make_unique< Model::EulerModel >( *model_opt );
    solver_ = std::make_unique< Solver::SphericalSolver >( *model_, Solver::SphericalSolver::SolverParameters() );
    
    auto tight_opt = Model::EulerModel::ComputeModel( 
        joint_x_tight_, 
        joint_y_tight_, 
        joint_z_tight_ );
    ASSERT_TRUE( tight_opt.has_value() );
    model_tight_ = std::make_unique< Model::EulerModel >( *tight_opt );
    solver_tight_ = std::make_unique< Solver::SphericalSolver >( *model_tight_, Solver::SphericalSolver::SolverParameters() );
}

void TearDown() override {}

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

std::vector< ConfigurationCase > AllConfigurations() const
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

Vec3d ApplyAngles(
    const Model::EulerModel model,
    const Vec3d& old_dir,
    const Vec3d& angles ) const
{
    const Vec3d a0 = model.GetJoint(0)->Axis();
    const Vec3d a1 = model.GetJoint(1)->Axis();
    const Vec3d a2 = model.GetJoint(2)->Axis();

    Mat3d R_total = (AngleAxis(angles[0], a0)
                   * AngleAxis(angles[1], a1)
                   * AngleAxis(angles[2], a2)).toRotationMatrix();

    return R_total * old_dir;
}

// ------------------------------------------------------------

void ExpectDirectionMatch(
    const Model::EulerModel model,
    const Vec3d& old_dir,
    const Vec3d& new_dir,
    const Solver::SphericalSolver::IKResult& result,
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
    std::optional<Vec3d> prefered_dir = std::nullopt )
{
    auto model = Model::EulerModel::ComputeModel(
        c.j1,
        c.j2,
        c.j3 );

    ASSERT_TRUE( model.has_value() )
        << "Failed ComputeModel for " << c.name;
    
    auto solver = Solver::SphericalSolver( *model, parameters_ );
    auto result = solver.Solve( old_dir, new_dir, prefered_dir );

    Vec3d recovered = ApplyAngles( *model, old_dir, result.angles );

    EXPECT_TRUE( recovered.isApprox( new_dir, 1e-4 ) )
        << "Config: " << c.name << " failed.\n"
        << "Expected dir: " << new_dir.transpose()    << "\n"
        << "Got dir:      " << recovered.transpose()  << "\n"
        << "Angles:       " << result.angles.transpose();
} 

// ------------------------------------------------------------

void CheckConfigurationForAngles( 
    ConfigurationCase c,     
    const Vec3d& old_dir,
    const Vec3d& angles,
    std::optional<Vec3d> prefered_dir = std::nullopt )
{
    auto model = Model::EulerModel::ComputeModel(
        c.j1,
        c.j2,
        c.j3 );

    ASSERT_TRUE( model.has_value() )
        << "Failed ComputeModel for " << c.name;
    
    Vec3d new_dir = ApplyAngles( *model, old_dir, angles );
    auto solver = Solver::SphericalSolver( *model, parameters_ );
    auto result = solver.Solve( old_dir, new_dir, prefered_dir );

    Vec3d recovered = ApplyAngles( *model, old_dir, result.angles );

    EXPECT_TRUE( recovered.isApprox( new_dir, 1e-4 ) )
        << "Config: " << c.name << " failed.\n"
        << "Expected dir: " << new_dir.transpose()    << "\n"
        << "Got dir:      " << recovered.transpose()  << "\n"
        << "Angles:       " << result.angles.transpose();
} 


// ------------------------------------------------------------

void CheckAllConfigurations( 
    const Vec3d& old_dir,
    const Vec3d& new_dir,
    std::optional<Vec3d> prefered_dir = std::nullopt )
{
    for ( const auto& c : AllConfigurations() )
        CheckConfiguration( c, old_dir, new_dir, prefered_dir );
} 

// ------------------------------------------------------------

void CheckAllConfigurationsForAngles( 
    const Vec3d& old_dir,
    const Vec3d& angles,
    std::optional<Vec3d> prefered_dir = std::nullopt )
{
    for ( const auto& c : AllConfigurations() )
        CheckConfigurationForAngles( c, old_dir, angles, prefered_dir );
} 

// ------------------------------------------------------------

Model::JointConstPtr joint_x_, joint_y_, joint_z_;
Model::JointConstPtr joint_x_tight_, joint_y_tight_, joint_z_tight_;

std::unique_ptr< Model::EulerModel > model_;
std::unique_ptr< Model::EulerModel > model_tight_;

std::unique_ptr< Solver::SphericalSolver > solver_;
std::unique_ptr< Solver::SphericalSolver > solver_tight_;
Solver::SphericalSolver::SolverParameters parameters_;
};

// ============================================================
// Solve (direction overload) — identity
// ============================================================

TEST_F( SphericalSolverTest, Solve_Dir_SameDirection_ReturnsNearZeroAngles )
{
    Vec3d old_dir = Vec3d::UnitZ();
    Vec3d new_dir = Vec3d::UnitZ();

    CheckAllConfigurations( old_dir, new_dir );
}

// ============================================================
// Solve (direction overload) — single axis rotations
// ============================================================

TEST_F( SphericalSolverTest, Solve_Dir_PureXRotation_DirectionMatches )
{
    Vec3d old_dir = Vec3d::UnitZ();
    // R_x(π/2) maps Z → -Y
    Vec3d new_dir = Vec3d( 0, -1, 0 );

    CheckAllConfigurations( old_dir, new_dir );
}

// ------------------------------------------------------------

TEST_F( SphericalSolverTest, Solve_Dir_PureYRotation_DirectionMatches )
{
    Vec3d old_dir = Vec3d::UnitZ();
    // R_y(π/2) maps Z → X
    Vec3d new_dir = Vec3d( 1, 0, 0 );

    CheckAllConfigurations( old_dir, new_dir );
}

// ------------------------------------------------------------

TEST_F( SphericalSolverTest, Solve_Dir_PureZRotation_NoDirectionChange )
{
    // Z rotation doesn't move UnitZ — error should be zero regardless of θ3
    Vec3d old_dir = Vec3d::UnitZ();
    Vec3d new_dir = Vec3d::UnitZ();

    CheckAllConfigurations( old_dir, new_dir );
}

// ============================================================
// Solve (direction overload) — combined rotations
// ============================================================

TEST_F( SphericalSolverTest, Solve_Dir_CombinedRotation_DirectionMatches )
{
    Vec3d old_dir = Vec3d::UnitX();
    // Build a target from known angles so we can verify round-trip
    const double a1 = M_PI / 4, a2 = M_PI / 6, a3 = 0.0;
    Vec3d angles = Vec3d( a1, a2, a3 );

   CheckAllConfigurationsForAngles( old_dir, angles );
}

// ------------------------------------------------------------

TEST_F( SphericalSolverTest, Solve_Dir_OppositeDirection_DirectionMatches )
{
    // 180° rotation — numerically tricky for FromTwoVectors
    Vec3d old_dir = Vec3d::UnitX();
    Vec3d new_dir = -Vec3d::UnitX();

    CheckAllConfigurations( old_dir, new_dir );
}

// ============================================================
// Solve (direction overload) — seed influence
// ============================================================

TEST_F( SphericalSolverTest, Solve_Dir_SeedCloserToResult_AnglesCloseToSeed )
{
    // There are infinitely many angle triplets that map old→new.
    // With a non-zero seed the optimizer should prefer angles near the seed.
    Vec3d old_dir = Vec3d::UnitZ();
    Vec3d new_dir = Vec3d( 0, -1, 0 );

    Vec3d seed_zero = Vec3d::Zero();
    Vec3d seed_far  = Vec3d( M_PI / 2, 0, 0 );   // close to the "canonical" solution

    auto result_zero = solver_->Solve( old_dir, new_dir, seed_zero );
    auto result_seed = solver_->Solve( old_dir, new_dir, seed_far );

    // Both must reach the target
    ExpectDirectionMatch( *model_, old_dir, new_dir, result_zero );
    ExpectDirectionMatch( *model_, old_dir, new_dir, result_seed );

    // Result seeded near the true solution should have smaller angle norm
    double dist_zero = ( result_zero.angles - seed_far ).norm();
    double dist_seed = ( result_seed.angles - seed_far ).norm();
    EXPECT_LE( dist_seed, dist_zero + epsilon );
}

// ============================================================
// Solve (direction overload) — joint limits respected
// ============================================================

TEST_F( SphericalSolverTest, Solve_Dir_TargetWithinLimits_AnglesSatisfyLimits )
{
    // Small rotation — should be reachable within tight limits
    Vec3d old_dir = Vec3d::UnitZ();
    Vec3d new_dir = ApplyAngles( *model_tight_, old_dir, Vec3d( M_PI / 8, M_PI / 8, 0 ) );
    Vec3d seed    = Vec3d::Zero();

    auto result = solver_tight_->Solve( old_dir, new_dir );
    ExpectDirectionMatch( *model_tight_, old_dir, new_dir, result, 1e-3 );

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

    auto result = solver_tight_->Solve( old_dir, new_dir );

    // Must not crash and angles must stay within limits (best-effort)
    for ( int i = 0; i < 3; ++i )
    {
        EXPECT_GE( result.angles[i], -M_PI / 6 - epsilon ) << "Joint " << i << " below min";
        EXPECT_LE( result.angles[i],  M_PI / 6 + epsilon ) << "Joint " << i << " above max";
    }
}

// ============================================================
// Solve (matrix overload)
// ============================================================

TEST_F( SphericalSolverTest, Solve_Mat_Identity_ReturnsNearZeroAngles )
{
//     Vec3d seed = Vec3d::Zero();

//     auto result = Solver::SphericalSolver::Solve( *model_, parameters_, seed, Mat3d::Identity() );

//     EXPECT_TRUE( result.solution.isZero( 1e-4 ) )
//         << "Angles: " << result.solution.transpose();
    EXPECT_TRUE( false );

}

// ------------------------------------------------------------

TEST_F( SphericalSolverTest, Solve_Mat_KnownRotation_ErrorNearZero )
{
    // Build R from known angles and verify the solver recovers them
    // const double a1 = M_PI / 4, a2 = M_PI / 6, a3 = 0.0;
    // Mat3d R_target = ( AngleAxis( a1, Vec3d::UnitZ() )
    //                  * AngleAxis( a2, Vec3d::UnitY() )
    //                  * AngleAxis( a3, Vec3d::UnitX() ) ).toRotationMatrix();

    // Vec3d seed = Vec3d::Zero();
    // auto result = Solver::SphericalSolver::Solve( *model_, parameters_, seed, R_target );

    // // The matrix overload uses a3 as the tracked direction.
    // // Error is defined on that direction, so check via ApplyAngles on UnitZ.
    // Vec3d old_dir = Vec3d::UnitZ();
    // Vec3d new_dir = R_target * old_dir;
    // ExpectDirectionMatch( old_dir, new_dir, result );
    EXPECT_TRUE( false );
}

// ============================================================
// Consistency — multiple calls same result
// ============================================================

TEST_F( SphericalSolverTest, Solve_Dir_Consistency_SameInputSameOutput )
{
    Vec3d old_dir = Vec3d( 1, 0, 0 ).normalized();
    Vec3d new_dir = Vec3d( 0, 1, 1 ).normalized();
    Vec3d seed    = Vec3d::Zero();

    auto r1 = solver_->Solve( old_dir, new_dir );
    auto r2 = solver_->Solve( old_dir, new_dir );
    
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

    Vec3d new_dir = ApplyAngles( *model_, old_dir, angles );

    auto result = solver_->Solve( old_dir, new_dir );

    Vec3d recovered = ApplyAngles( *model_, old_dir, result.angles );
    EXPECT_TRUE( recovered.isApprox( new_dir, 1e-3 ) ) 
            << "Original Angles " << angles.transpose() << "\n"
            << "Expected:  " << new_dir.transpose()   << "\n"
            << "Got:       " << recovered.transpose() << "\n"
            << "Result Angles:    " << result.angles.transpose();
}

TEST_F( SphericalSolverTest, Solve_Dir_RandomDirections_DirectionAlwaysMatches )
{
    random_numbers::RandomNumberGenerator rng;
    Vec3d old_dir = Vec3d::UnitX();
    //Vec3d old_dir = Vec3d::Ones();
    Vec3d seed    = Vec3d::Zero();

    const int ITER = 20;
    int failures = 0;

    const auto& limit0 = model_->GetJoint(0)->GetLimits();
    const auto& limit1 = model_->GetJoint(1)->GetLimits();
    const auto& limit2 = model_->GetJoint(2)->GetLimits();

    Vec3d random_angles;

    for ( int i = 0; i < ITER; ++i )
    {
        limit0.Random( rng, &random_angles[0] );
        limit1.Random( rng, &random_angles[1] );
        limit2.Random( rng, &random_angles[2] );

        Vec3d new_dir = ApplyAngles( *model_, old_dir, random_angles );

        auto result = solver_->Solve( old_dir, new_dir );

        Vec3d recovered = ApplyAngles( *model_, old_dir, result.angles );
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