#include "Euler/EulerModel.hpp"

#include "Global.hpp"

#include "KinematicTestBase.hpp"
#include "Model/Joint/Joint.hpp"

#include <gtest/gtest.h>

namespace SOArm100::Kinematics::Test
{

// ============================================================
// EulerModelTest
// ============================================================

class EulerModelTest : public KinematicTestBase
{
protected:

void SetUp() override
{
    joint_x_ = MakeRevoluteJoint(
        Vec3d::UnitX(),
        Vec3d::Zero(),
        -M_PI,
        M_PI );

    joint_y_ = MakeRevoluteJoint(
        Vec3d::UnitY(),
        Vec3d::Zero(),
        -M_PI,
        M_PI );

    joint_z_ = MakeRevoluteJoint(
        Vec3d::UnitZ(),
        Vec3d::Zero(),
        -M_PI,
        M_PI );

    joint_nonorth_1_ = MakeRevoluteJoint(
        Vec3d( 1.0, 0.5, 0.0 ).normalized(),
        Vec3d::Zero(),
        -M_PI,
        M_PI );

    joint_nonorth_2_ = MakeRevoluteJoint(
        Vec3d( 0.0, 1.0, 0.5 ).normalized(),
        Vec3d::Zero(),
        -M_PI,
        M_PI );
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

Mat3d MakeBasis(
    const Vec3d& a1,
    const Vec3d& a2,
    const Vec3d& a3 ) const
{
    Mat3d P;

    P.col( 0 ) = a1;
    P.col( 1 ) = a2;
    P.col( 2 ) = a3;

    if ( P.determinant() < 0 )
        P.col( 0 ) *= -1.0;

    return P;
}

// ------------------------------------------------------------

Mat3d ComposeRotation(
    const Model::EulerModel& model,
    double a1,
    double a2,
    double a3,
    const Vec3d& ax1,
    const Vec3d& ax2,
    const Vec3d& ax3 ) const
{
    return ( AngleAxis( a1, ax1 ) *
             AngleAxis( a2, ax2 ) *
             AngleAxis( a3, ax3 ) ).toRotationMatrix();
}

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

void ExpectRoundTrip(
    const ConfigurationCase& c,
    double a1,
    double a2,
    double a3 )
{
    auto model = Model::EulerModel::ComputeModel(
        c.j1,
        c.j2,
        c.j3 );

    ASSERT_TRUE( model.has_value() )
        << "Failed ComputeModel for " << c.name;

    Mat3d R_world = ComposeRotation(
        *model,
        a1,
        a2,
        a3,
        c.ax1,
        c.ax2,
        c.ax3 );
    
    bool singularity;
    auto branches =
        model->DecomposePhysical( R_world );

    EXPECT_EQ( branches.size(), 2 );
    
    Vec3d angles = branches[0];
    Mat3d R_reconstructed = ComposeRotation( 
        *model, 
        angles[0],
        angles[1],
        angles[2],
        c.ax1,
        c.ax2,
        c.ax3 );

    EXPECT_TRUE(
        R_world.isApprox(
            R_reconstructed,
            1e-6 ) )
        << "Reconstruction failed for "
        << c.name
        << "\nExpected:\n"
        << R_world
        << "\nGot:\n"
        << R_reconstructed
        << "\nAngles: "
        << angles.transpose();

    angles = branches[1];
    R_reconstructed = ComposeRotation( 
        *model, 
        angles[0],
        angles[1],
        angles[2],
        c.ax1,
        c.ax2,
        c.ax3 );

    EXPECT_TRUE(
        R_world.isApprox(
            R_reconstructed,
            1e-6 ) )
        << "Reconstruction failed for "
        << c.name
        << "\nExpected:\n"
        << R_world
        << "\nGot:\n"
        << R_reconstructed
        << "\nAngles: "
        << angles.transpose();
}

// ------------------------------------------------------------

Model::JointConstPtr joint_x_;
Model::JointConstPtr joint_y_;
Model::JointConstPtr joint_z_;

Model::JointConstPtr joint_nonorth_1_;
Model::JointConstPtr joint_nonorth_2_;
};

// ============================================================
// ComputeModel
// ============================================================

TEST_F(
    EulerModelTest,
    ComputeModel_AllConfigurations_ReturnValidModels )
{
    for ( const auto& c : AllConfigurations() )
    {
        auto model =
            Model::EulerModel::ComputeModel(
                c.j1,
                c.j2,
                c.j3 );

        EXPECT_TRUE( model.has_value() )
            << "Failed for "
            << c.name;
    }
}

// ------------------------------------------------------------

TEST_F(
    EulerModelTest,
    ComputeModel_NonOrthogonalAxes_ReturnsNullopt )
{
    auto model =
        Model::EulerModel::ComputeModel(
            joint_nonorth_1_,
            joint_nonorth_2_,
            joint_z_ );

    EXPECT_FALSE( model.has_value() );
}

// ------------------------------------------------------------

TEST_F(
    EulerModelTest,
    ComputeModel_DuplicateAxes_ReturnsNullopt )
{
    auto model =
        Model::EulerModel::ComputeModel(
            joint_x_,
            joint_x_,
            joint_z_ );

    EXPECT_FALSE( model.has_value() );
}

// ============================================================
// Identity
// ============================================================

TEST_F(
    EulerModelTest,
    DecomposeEulerAngles_Identity_ReturnsZero )
{
    for ( const auto& c : AllConfigurations() )
    {
        auto model =
            Model::EulerModel::ComputeModel(
                c.j1,
                c.j2,
                c.j3 );

        ASSERT_TRUE( model.has_value() );

        bool singularity;
        auto branches =
            model->DecomposePhysical( Mat3d::Identity() );
        
        EXPECT_EQ( branches.size(), 2 );

        Vec3d angles = branches[0];
        EXPECT_NEAR( angles[0], 0.0, 1e-6 )
            << c.name;

        EXPECT_NEAR( angles[1], 0.0, 1e-6 )
            << c.name;

        EXPECT_NEAR( angles[2], 0.0, 1e-6 )
            << c.name;

        angles = branches[1];
        EXPECT_NEAR( angles[0], M_PI, 1e-6 )
            << c.name;

        EXPECT_NEAR( angles[1], M_PI, 1e-6 )
            << c.name;

        EXPECT_NEAR( angles[2], M_PI, 1e-6 )
            << c.name;
    }
}

// ============================================================
// Pure axis tests
// ============================================================

TEST_F(
    EulerModelTest,
    DecomposeEulerAngles_PureFirstAxis )
{
    for ( const auto& c : AllConfigurations() )
    {
        ExpectRoundTrip(
            c,
            0.4,
            0.0,
            0.0 );
    }
}

// ------------------------------------------------------------

TEST_F(
    EulerModelTest,
    DecomposeEulerAngles_PureSecondAxis )
{
    for ( const auto& c : AllConfigurations() )
    {
        ExpectRoundTrip(
            c,
            0.0,
            -0.5,
            0.0 );
    }
}

// ------------------------------------------------------------

TEST_F(
    EulerModelTest,
    DecomposeEulerAngles_PureThirdAxis )
{
    for ( const auto& c : AllConfigurations() )
    {
        ExpectRoundTrip(
            c,
            0.0,
            0.0,
            0.7 );
    }
}

// ============================================================
// Round-trip tests
// ============================================================

TEST_F(
    EulerModelTest,
    DecomposeEulerAngles_AllConfigurations_RoundTrip_1 )
{
    for ( const auto& c : AllConfigurations() )
    {
        ExpectRoundTrip(
            c,
            0.3,
            -0.4,
            0.2 );
    }
}

// ------------------------------------------------------------

TEST_F(
    EulerModelTest,
    DecomposeEulerAngles_AllConfigurations_RoundTrip_2 )
{
    for ( const auto& c : AllConfigurations() )
    {
        ExpectRoundTrip(
            c,
            -0.8,
            0.25,
            -0.6 );
    }
}

// ------------------------------------------------------------

TEST_F(
    EulerModelTest,
    DecomposeEulerAngles_AllConfigurations_RoundTrip_3 )
{
    for ( const auto& c : AllConfigurations() )
    {
        ExpectRoundTrip(
            c,
            M_PI / 4.0,
            -M_PI / 6.0,
            M_PI / 5.0 );
    }
}

// ------------------------------------------------------------

TEST_F(
    EulerModelTest,
    DecomposeEulerAngles_AllConfigurations_RandomRoundTrip )
{
    random_numbers::RandomNumberGenerator rng;

    const int ITER = 20;
    int failures = 0;

    for ( int i = 0; i < ITER; ++i )
    {
        // Random direction on unit sphere
        Vec3d angles = Vec3d(
            rng.uniformReal( -M_PI, M_PI ),
            rng.uniformReal( -M_PI, M_PI ),
            rng.uniformReal( -M_PI, M_PI ) );

        for ( const auto& c : AllConfigurations() )
        {
            ExpectRoundTrip(
                c,
                angles[0],
                angles[1],
                angles[2] );
        }
    }
}

// ============================================================
// Near gimbal lock
// ============================================================

TEST_F(
    EulerModelTest,
    DecomposeEulerAngles_NearGimbalLock )
{
    for ( const auto& c : AllConfigurations() )
    {
        ExpectRoundTrip(
            c,
            0.2,
            M_PI / 2.0 - 1e-4,
            -0.3 );
    }
}

// ============================================================
// GetJoints
// ============================================================

TEST_F(
    EulerModelTest,
    GetJoints_ReturnsCorrectOrder )
{
    auto model =
        Model::EulerModel::ComputeModel(
            joint_x_,
            joint_y_,
            joint_z_ );

    ASSERT_TRUE( model.has_value() );

    const auto& joints =
        model->GetJoints();

    ASSERT_EQ( joints.size(), 3 );

    EXPECT_EQ( joints[0], joint_x_ );
    EXPECT_EQ( joints[1], joint_y_ );
    EXPECT_EQ( joints[2], joint_z_ );
}

// ------------------------------------------------------------

} // namespace SOArm100::Kinematics::Test