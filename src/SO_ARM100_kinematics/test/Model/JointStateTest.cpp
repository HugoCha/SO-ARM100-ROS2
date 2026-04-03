#include "Model/JointState.hpp"

#include "Global.hpp"
#include "Model/Joint.hpp"
#include "Model/Limits.hpp"
#include "Utils/Converter.hpp"

#include <gtest/gtest.h>
#include <memory>

namespace SOArm100::Kinematics::Test
{

// ============================================================
// Helpers
// ============================================================

static Model::JointConstPtr MakeRevoluteJoint(
	const Vec3d& axis,
	const Vec3d& origin,
	double min   = -M_PI,
	double max   =  M_PI )
{
	return std::make_shared< const Model::Joint >(
		Model::Twist( axis, origin ),
		Model::Link( ToTransformMatrix( origin ), 0 ),
		Model::Limits( min, max )
		);
}

static Model::JointConstPtr MakePrismaticJoint(
	const Vec3d& axis,
	const Vec3d& origin,
	double min = 0.0,
	double max = 1.0 )
{
	return std::make_shared< const Model::Joint >(
		Model::Twist( axis ),
		Model::Link( ToTransformMatrix( origin ), 0 ),
		Model::Limits( min, max ) );
}

// ============================================================
// Fixture
// ============================================================

class JointStateTest : public ::testing::Test
{
protected:
void SetUp() override
{
	revolute_joint_   = MakeRevoluteJoint(  Vec3d( 0, 0, 1 ), Vec3d( 0, 0, 0 ) );
	prismatic_joint_  = MakePrismaticJoint( Vec3d( 0, 0, 1 ), Vec3d( 0, 0, 0 ), 0.0, 1.0 );

	revolute_state_   = std::make_shared< Model::JointState >( revolute_joint_  );
	prismatic_state_  = std::make_shared< Model::JointState >( prismatic_joint_ );
}

void TearDown() override {
}

Model::JointConstPtr revolute_joint_;
Model::JointConstPtr prismatic_joint_;

std::shared_ptr< Model::JointState > revolute_state_;
std::shared_ptr< Model::JointState > prismatic_state_;
};

// ============================================================
// Constructor
// ============================================================

TEST_F( JointStateTest, Constructor_InitialOriginMatchesJoint )
{
	EXPECT_TRUE( revolute_state_->Origin().isApprox( revolute_joint_->Origin() ) )
	    << "Initial origin should match the joint's origin";
}

// ------------------------------------------------------------

TEST_F( JointStateTest, Constructor_InitialAxisMatchesJoint )
{
	EXPECT_TRUE( revolute_state_->Axis().isApprox( revolute_joint_->Axis() ) )
	    << "Initial axis should match the joint's axis";
}

// ------------------------------------------------------------

TEST_F( JointStateTest, Constructor_InitialValueIsZeroWhenWithinLimits )
{
	// Limits are [-pi, pi], so 0 is within range
	EXPECT_DOUBLE_EQ( revolute_state_->Value(), 0.0 )
	    << "Initial value should be 0 when 0 is within limits";
}

// ------------------------------------------------------------

TEST_F( JointStateTest, Constructor_InitialValueIsCenterWhenZeroOutsideLimits )
{
	// Limits [0.5, 1.0] — 0 is outside, centre should be used
	auto joint = MakeRevoluteJoint( Vec3d( 0, 0, 1 ), Vec3d( 0, 0, 0 ), 0.5, 1.0 );
	Model::JointState state( joint );

	double expected = ( 0.5 + 1.0 ) / 2.0;
	EXPECT_DOUBLE_EQ( state.Value(), expected )
	    << "Initial value should be the centre of limits when 0 is outside";
}

// ------------------------------------------------------------

TEST_F( JointStateTest, Constructor_FlagsAreClean )
{
	EXPECT_FALSE( revolute_state_->UpdateValueRequired() )
	    << "update_value_required_ should be false after construction";
	EXPECT_FALSE( revolute_state_->UpdatePoseRequired() )
	    << "update_pose_required_ should be false after construction";
}

// ============================================================
// SetState
// ============================================================

TEST_F( JointStateTest, SetState_UpdatesOrigin )
{
	Vec3d new_origin( 1, 2, 3 );
	revolute_state_->SetState( new_origin, Vec3d( 0, 0, 1 ), 0.5 );
	EXPECT_TRUE( revolute_state_->Origin().isApprox( new_origin ) );
}

// ------------------------------------------------------------

TEST_F( JointStateTest, SetState_UpdatesAxis )
{
	Vec3d new_axis = Vec3d( 0, 1, 0 ).normalized();
	revolute_state_->SetState( Vec3d( 0, 0, 0 ), new_axis, 0.5 );
	EXPECT_TRUE( revolute_state_->Axis().isApprox( new_axis ) );
}

// ------------------------------------------------------------

TEST_F( JointStateTest, SetState_UpdatesValue )
{
	double new_value = 1.2;
	revolute_state_->SetState( Vec3d( 0, 0, 0 ), Vec3d( 0, 0, 1 ), new_value );
	EXPECT_DOUBLE_EQ( revolute_state_->Value(), new_value );
}

// ------------------------------------------------------------

TEST_F( JointStateTest, SetState_DoesNotSetUpdateFlag )
{
	revolute_state_->SetState( Vec3d( 1, 0, 0 ), Vec3d( 0, 0, 1 ), 0.3 );
	// SetState is an authoritative assignment — no deferred update expected
	EXPECT_FALSE( revolute_state_->UpdateValueRequired() );
}

// ============================================================
// SetPose
// ============================================================

TEST_F( JointStateTest, SetPose_UpdatesOriginAndAxis )
{
	Vec3d new_origin( 0, 1, 0 );
	Vec3d new_axis( 1, 0, 0 );
	revolute_state_->SetPose( new_origin, new_axis );

	EXPECT_TRUE( revolute_state_->Origin().isApprox( new_origin ) );
	EXPECT_TRUE( revolute_state_->Axis().isApprox( new_axis ) );
}

// ------------------------------------------------------------

TEST_F( JointStateTest, SetPose_SetsUpdateValueRequired )
{
	revolute_state_->SetPose( Vec3d( 0, 1, 0 ), Vec3d( 1, 0, 0 ) );
	EXPECT_TRUE( revolute_state_->UpdateValueRequired() )
	    << "SetPose should mark value as needing update";
}

// ------------------------------------------------------------

TEST_F( JointStateTest, SetPose_ClearsUpdatePoseRequired )
{
	revolute_state_->SetPose( Vec3d( 0, 1, 0 ), Vec3d( 1, 0, 0 ) );
	EXPECT_FALSE( revolute_state_->UpdatePoseRequired() )
	    << "SetPose should clear the pose-update flag";
}

// ============================================================
// UpdatePose
// ============================================================

TEST_F( JointStateTest, UpdatePose_UpdatesOriginAndAxis )
{
	Vec3d new_origin( 3, 2, 1 );
	Vec3d new_axis( 0, 1, 0 );
	revolute_state_->UpdatePose( new_origin, new_axis );

	EXPECT_TRUE( revolute_state_->Origin().isApprox( new_origin ) );
	EXPECT_TRUE( revolute_state_->Axis().isApprox( new_axis ) );
}

// ------------------------------------------------------------

TEST_F( JointStateTest, UpdatePose_ClearsUpdatePoseRequired )
{
	revolute_state_->UpdatePose( Vec3d( 0, 0, 0 ), Vec3d( 0, 0, 1 ) );
	EXPECT_FALSE( revolute_state_->UpdatePoseRequired() )
	    << "UpdatePose should clear the pose-update flag";
}

// ============================================================
// UpdateValue — Revolute
// ============================================================

TEST_F( JointStateTest, UpdateValue_Revolute_PositiveRotation )
{
	// Joint axis along Z; rotate child direction from +X towards +Y (90 degrees CCW)
	Vec3d old_dir = Vec3d( 1, 0, 0 );
	Vec3d new_dir = Vec3d( 0, 1, 0 );

	revolute_state_->UpdateValue( Vec3d( 0, 0, 0 ), old_dir, new_dir );

	EXPECT_NEAR( revolute_state_->Value(), M_PI / 2.0, 1e-9 );
	EXPECT_FALSE( revolute_state_->UpdateValueRequired() );
}

// ------------------------------------------------------------

TEST_F( JointStateTest, UpdateValue_Revolute_NegativeRotation )
{
	Vec3d old_dir = Vec3d( 1, 0, 0 );
	Vec3d new_dir = Vec3d( 0, -1, 0 );

	revolute_state_->UpdateValue( Vec3d( 0, 0, 0 ), old_dir, new_dir );

	EXPECT_NEAR( revolute_state_->Value(), -M_PI / 2.0, 1e-9 );
}

// ------------------------------------------------------------

TEST_F( JointStateTest, UpdateValue_Revolute_ZeroAngle )
{
	Vec3d dir = Vec3d( 1, 0, 0 );
	revolute_state_->UpdateValue( Vec3d( 0, 0, 0 ), dir, dir );

	EXPECT_NEAR( revolute_state_->Value(), 0.0, 1e-9 );
}

// ------------------------------------------------------------

TEST_F( JointStateTest, UpdateValue_Revolute_AccumulatesValues )
{
	Vec3d old_dir = Vec3d( 1, 0, 0 );
	Vec3d mid_dir = Vec3d( 0, 1, 0 );
	Vec3d new_dir = Vec3d( -1, 0, 0 );

	revolute_state_->UpdateValue( Vec3d( 0, 0, 0 ), old_dir, mid_dir );
	revolute_state_->UpdateValue( Vec3d( 0, 0, 0 ), mid_dir, new_dir );

	EXPECT_NEAR( revolute_state_->Value(), M_PI, 1e-9 );
}

// ------------------------------------------------------------

TEST_F( JointStateTest, UpdateValue_Revolute_ClampsToLimitAndSetsFlag )
{
	// Limits [-pi, pi]; apply a rotation that would exceed +pi
	// Start at +pi - 0.1, then rotate by +0.5 → would be +pi + 0.4 → clamped to +pi
	revolute_state_->SetState( Vec3d( 0, 0, 0 ), Vec3d( 0, 0, 1 ), M_PI - 0.1 );

	Vec3d old_dir = Vec3d( 1, 0, 0 );
	// Rotate 0.5 rad CCW (positive around Z)
	Vec3d new_dir = Vec3d( std::cos( 0.5 ), std::sin( 0.5 ), 0 );

	revolute_state_->UpdateValue( Vec3d( 0, 0, 0 ), old_dir, new_dir );

	EXPECT_DOUBLE_EQ( revolute_state_->Value(), M_PI )
	    << "Value should be clamped to upper limit";
	EXPECT_TRUE( revolute_state_->UpdatePoseRequired() )
	    << "UpdatePoseRequired should be set when value is clamped";
}

// ------------------------------------------------------------

TEST_F( JointStateTest, UpdateValue_Revolute_ParallelDirectionToAxisIsNoOp )
{
	// Both old and new directions are parallel to the joint axis (Z)
	// The projected vectors will be near-zero → early return expected
	Vec3d parallel( 0, 0, 1 );
	double value_before = revolute_state_->Value();

	revolute_state_->UpdateValue( Vec3d( 0, 0, 0 ), parallel, parallel );

	EXPECT_DOUBLE_EQ( revolute_state_->Value(), value_before )
	    << "Value should not change when projected directions are zero";
	EXPECT_FALSE( revolute_state_->UpdateValueRequired() );
}

// ============================================================
// UpdateValue — Prismatic
// ============================================================

TEST_F( JointStateTest, UpdateValue_Prismatic_PositiveDelta )
{
	// Move origin from (0,0,0) to (0,0,0.3); axis is +Z → delta = 0.3
	prismatic_state_->UpdatePose( Vec3d( 0, 0, 0.3 ), Vec3d( 0, 0, 1 ) );
	prismatic_state_->UpdateValue( Vec3d( 0, 0, 0 ), Vec3d(), Vec3d() );

	EXPECT_NEAR( prismatic_state_->Value(), 0.3, 1e-9 );
	EXPECT_FALSE( prismatic_state_->UpdateValueRequired() );
}

// ------------------------------------------------------------

TEST_F( JointStateTest, UpdateValue_Prismatic_NegativeDelta )
{
	// Start at value 0.5, move origin back by 0.2 along axis
	prismatic_state_->SetState( Vec3d( 0, 0, 0.5 ), Vec3d( 0, 0, 1 ), 0.5 );
	prismatic_state_->UpdatePose( Vec3d( 0, 0, 0.3 ), Vec3d( 0, 0, 1 ) );
	prismatic_state_->UpdateValue( Vec3d( 0, 0, 0.5 ), Vec3d(), Vec3d() );

	EXPECT_NEAR( prismatic_state_->Value(), 0.3, 1e-9 );
}

// ------------------------------------------------------------

TEST_F( JointStateTest, UpdateValue_Prismatic_ClampsToUpperLimitAndSetsFlag )
{
	// Limits [0, 1]; start at 0.8, move by 0.5 → would be 1.3 → clamped to 1.0
	prismatic_state_->SetState( Vec3d( 0, 0, 0.8 ), Vec3d( 0, 0, 1 ), 0.8 );
	prismatic_state_->UpdatePose( Vec3d( 0, 0, 1.3 ), Vec3d( 0, 0, 1 ) );
	prismatic_state_->UpdateValue( Vec3d( 0, 0, 0.8 ), Vec3d(), Vec3d() );

	EXPECT_DOUBLE_EQ( prismatic_state_->Value(), 1.0 )
	    << "Value should be clamped to upper limit";
	EXPECT_TRUE( prismatic_state_->UpdatePoseRequired() )
	    << "UpdatePoseRequired should be set when value is clamped";
}

// ------------------------------------------------------------

TEST_F( JointStateTest, UpdateValue_Prismatic_ClampsToLowerLimitAndSetsFlag )
{
	// Limits [0, 1]; start at 0.2, move by -0.5 → would be -0.3 → clamped to 0.0
	prismatic_state_->SetState( Vec3d( 0, 0, 0.2 ), Vec3d( 0, 0, 1 ), 0.2 );
	prismatic_state_->UpdatePose( Vec3d( 0, 0, -0.3 ), Vec3d( 0, 0, 1 ) );
	prismatic_state_->UpdateValue( Vec3d( 0, 0, 0.2 ), Vec3d(), Vec3d() );

	EXPECT_DOUBLE_EQ( prismatic_state_->Value(), 0.0 )
	    << "Value should be clamped to lower limit";
	EXPECT_TRUE( prismatic_state_->UpdatePoseRequired() );
}

// ------------------------------------------------------------

TEST_F( JointStateTest, UpdateValue_Prismatic_PerpendicularMovementIsZeroDelta )
{
	// Move perpendicular to axis (X direction); axis is Z → dot product = 0
	prismatic_state_->SetState( Vec3d( 0, 0, 0.5 ), Vec3d( 0, 0, 1 ), 0.5 );
	prismatic_state_->UpdatePose( Vec3d( 1, 0, 0.5 ), Vec3d( 0, 0, 1 ) );
	prismatic_state_->UpdateValue( Vec3d( 0, 0, 0.5 ), Vec3d(), Vec3d() );

	EXPECT_NEAR( prismatic_state_->Value(), 0.5, 1e-9 )
	    << "Perpendicular movement should not change the prismatic value";
}

// ============================================================
// ClearsUpdateValueRequired after UpdateValue
// ============================================================

TEST_F( JointStateTest, UpdateValue_ClearsUpdateValueRequired )
{
	// SetPose marks update_value_required_
	revolute_state_->SetPose( Vec3d( 0, 1, 0 ), Vec3d( 0, 0, 1 ) );
	ASSERT_TRUE( revolute_state_->UpdateValueRequired() );

	revolute_state_->UpdateValue( Vec3d( 0, 0, 0 ), Vec3d( 1, 0, 0 ), Vec3d( 1, 0, 0 ) );
	EXPECT_FALSE( revolute_state_->UpdateValueRequired() );
}

// ------------------------------------------------------------

}