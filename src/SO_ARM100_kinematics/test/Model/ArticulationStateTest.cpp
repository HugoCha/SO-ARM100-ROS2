#include "Model/Skeleton/ArticulationState.hpp"

#include "Global.hpp"

#include "Model/Joint/Joint.hpp"
#include "Model/Limits.hpp"
#include "Model/Skeleton/Articulation.hpp"
#include "Model/Skeleton/ArticulationType.hpp"
#include "Model/Skeleton/Bone.hpp"
#include "Model/Skeleton/BoneState.hpp"
#include "Utils/Converter.hpp"
#include "Utils/KinematicsUtils.hpp"
#include "Utils/StringConverter.hpp"

#include <gtest/gtest.h>
#include <memory>
#include <cmath>
#include <ostream>

namespace SOArm100::Kinematics::Test
{

// ============================================================
// Helpers
// ============================================================

static Model::JointConstPtr MakeRevoluteJoint(
	const Vec3d& axis,
	const Vec3d& origin,
	double min   = -M_PI / 2,
	double max   =  M_PI / 2 )
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

class ArticulationStateTest : public ::testing::Test
{
protected:
void SetUp() override
{
	// Revolute articulation (single joint)
	revolute_joint_ = MakeRevoluteJoint( Vec3d( 0, 0, 1 ), Vec3d( 1, 0, 0 ) );
	std::vector< Model::JointConstPtr > revolute_joints = { revolute_joint_ };
	revolute_articulation_ = std::make_shared< const Model::Articulation >(
		Model::ArticulationType::Revolute,
		revolute_joints,
		Vec3d( 1, 0, 0 ) );

	// Prismatic articulation (single joint)
	prismatic_joint_ = MakePrismaticJoint( Vec3d( 0, 0, 1 ), Vec3d( 0, 0, 0 ), 0.0, 1.0 );
	std::vector< Model::JointConstPtr > prismatic_joints = { prismatic_joint_ };
	prismatic_articulation_ = std::make_shared< const Model::Articulation >(
		Model::ArticulationType::Prismatic,
		prismatic_joints,
		Vec3d( 0, 0, 0 ) );

	// Universal articulation (two joints)
	auto joint1 = MakeRevoluteJoint( Vec3d( 1, 0, 0 ), Vec3d( 0, 0, 0 ) );
	auto joint2 = MakeRevoluteJoint( Vec3d( 0, 1, 0 ), Vec3d( 0.1, 0.1, 0 ) );
	std::vector< Model::JointConstPtr > universal_joints = { joint1, joint2 };
	universal_articulation_ = std::make_shared< const Model::Articulation >(
		Model::ArticulationType::Universal,
		universal_joints,
		Vec3d( 0.1, 0, 0 ) );

	// Spherical articulation (three joints)
	auto joint_s1 = MakeRevoluteJoint( Vec3d( 1, 0, 0 ), Vec3d( 0.1, 0, 0 ) );
	auto joint_s2 = MakeRevoluteJoint( Vec3d( 0, 1, 0 ), Vec3d( 0, 0, 0 ) );
	auto joint_s3 = MakeRevoluteJoint( Vec3d( 0, 0, 1 ), Vec3d( 0, 0, 0.1 ) );
	std::vector< Model::JointConstPtr > spherical_joints = { joint_s1, joint_s2, joint_s3 };
	spherical_articulation_ = std::make_shared< const Model::Articulation >(
		Model::ArticulationType::Spherical,
		spherical_joints,
		Vec3d( 0, 0, 0 ) );
}

void TearDown() override {
}

Model::JointConstPtr revolute_joint_;
Model::JointConstPtr prismatic_joint_;

Model::ArticulationConstPtr revolute_articulation_;
Model::ArticulationConstPtr prismatic_articulation_;
Model::ArticulationConstPtr universal_articulation_;
Model::ArticulationConstPtr spherical_articulation_;
};

// ============================================================
// Constructor
// ============================================================

TEST_F( ArticulationStateTest, Constructor_InitialOriginMatchesArticulationCenter )
{
    auto revolute_state = Model::ArticulationState( revolute_articulation_ );
	EXPECT_TRUE( revolute_state.Origin().isApprox( revolute_articulation_->Center() ) )
	    << "Initial origin should match the articulation's center";
}

// ------------------------------------------------------------

TEST_F( ArticulationStateTest, Constructor_InitialAxisMatchesLastJointAxis )
{
	// For revolute, the axis should be the revolute joint's axis
    auto revolute_state = Model::ArticulationState( revolute_articulation_ );
	EXPECT_TRUE( revolute_state.Axis().isApprox( revolute_joint_->Axis() ) )
	    << "Axis should match the last joint's axis";
}

// ------------------------------------------------------------

TEST_F( ArticulationStateTest, Constructor_GetArticulationReturnsCorrectArticulation )
{
    auto revolute_state = Model::ArticulationState( revolute_articulation_ );
	EXPECT_EQ( revolute_state.GetArticulation(), revolute_articulation_ )
	    << "GetArticulation should return the same articulation passed to constructor";
}

// ============================================================
// GetJointValues
// ============================================================

TEST_F( ArticulationStateTest, GetJointValues_ReturnsVectorOfDefaultValues )
{
    auto revolute_state = Model::ArticulationState( revolute_articulation_ );
	auto values = revolute_state.GetJointValues();
	EXPECT_EQ( values.size(), 1 )
	    << "Revolute articulation should have 1 joint";
	EXPECT_DOUBLE_EQ( values[0], 0.0 )
	    << "Default joint value should be 0";
}

// ------------------------------------------------------------

TEST_F( ArticulationStateTest, GetJointValues_UniversalReturnsVectorOfTwoValues )
{
    auto universal_state = Model::ArticulationState( universal_articulation_ );
	auto values = universal_state.GetJointValues();
	EXPECT_EQ( values.size(), 2 )
	    << "Universal articulation should have 2 joints";
    EXPECT_DOUBLE_EQ( values[0], 0.0 );
    EXPECT_DOUBLE_EQ( values[1], 0.0 );
}

// ------------------------------------------------------------

TEST_F( ArticulationStateTest, GetJointValues_SphericalReturnsVectorOfTwoValues )
{
    auto spherical_state = Model::ArticulationState( spherical_articulation_ );
	auto values = spherical_state.GetJointValues();
	EXPECT_EQ( values.size(), 3 )
	    << "Universal articulation should have 3 joints";
    EXPECT_DOUBLE_EQ( values[0], 0.0 );
    EXPECT_DOUBLE_EQ( values[1], 0.0 );
    EXPECT_DOUBLE_EQ( values[2], 0.0 );
}

// ============================================================
// SetState
// ============================================================

TEST_F( ArticulationStateTest, SetState_RevoluteWithRotation_UpdatesPoseAndValueCorrectly )
{
	Vec3d new_origin( 1, 0, 0 );
	Vec3d new_axis( 0, 1, 0 );

	Quaternion rotation = Quaternion::FromTwoVectors( Vec3d( 0, 0, 1 ), Vec3d( 0, 1, 0 ) );
	VecXd values = VecXd::Zero( 1 );

    auto revolute_state = Model::ArticulationState( revolute_articulation_ );
	revolute_state.SetState( rotation, new_origin, values );

    EXPECT_EQ( 0, revolute_state.Value() );
	EXPECT_TRUE( revolute_state.Origin().isApprox( new_origin ) )
        << "Expected = " << new_origin.transpose() << std::endl
        << "Result   = " << revolute_state.Origin().transpose() << std::endl;
	EXPECT_TRUE( revolute_state.Axis().isApprox( new_axis ) )
        << "Expected = " << new_axis.transpose() << std::endl
        << "Result   = " << revolute_state.Axis().transpose() << std::endl;
}

// ------------------------------------------------------------

TEST_F( ArticulationStateTest, SetState_UniversalWithRotation_UpdatesPoseAndValueCorrectly )
{
	Vec3d new_origin( 1, 0, 0 );
	Quaternion rotation = Quaternion::FromTwoVectors( Vec3d( 0, 0, 1 ), Vec3d( 0, 1, 0 ) );
	VecXd values = VecXd::Zero( 2 );

    auto universal_state = Model::ArticulationState( universal_articulation_ );
	universal_state.SetState( rotation, new_origin, values );

	EXPECT_EQ( 0, universal_state.Value() );
	EXPECT_TRUE( universal_state.Origin().isApprox( Vec3d( 1, 0, 0 ) ) )
        << "Expected = " << Vec3d( 1, 0, 0 ).transpose() << std::endl
        << "Result   = " << universal_state.Origin().transpose() << std::endl;
	EXPECT_TRUE( universal_state.Axis().isApprox( Vec3d( 0, 0, -1 ) ) )
        << "Expected = " << Vec3d( 0, 0, -1 ).transpose() << std::endl
        << "Result   = " << universal_state.Axis().transpose() << std::endl;
}

// ------------------------------------------------------------

TEST_F( ArticulationStateTest, SetState_SphericalWithRotation_UpdatesPoseAndValueCorrectly )
{
	Vec3d new_origin( 1, 0, 0 );
    Vec3d new_axis( 0, 1, 0 );
	Quaternion rotation = Quaternion::FromTwoVectors( Vec3d( 0, 0, 1 ), Vec3d( 0, 1, 0 ) );
	VecXd values = VecXd::Zero( 3 );

    auto spherical_state = Model::ArticulationState( spherical_articulation_ );
	spherical_state.SetState( rotation, new_origin, values );

    EXPECT_EQ( spherical_state.Value(), 0 );
	EXPECT_TRUE( spherical_state.Origin().isApprox( new_origin ) )
        << "Expected = " << new_origin.transpose() << std::endl
        << "Result   = " << spherical_state.Origin().transpose() << std::endl;
	EXPECT_TRUE( spherical_state.Axis().isApprox( new_axis ) )
        << "Expected = " << new_axis.transpose() << std::endl
        << "Result   = " << spherical_state.Axis().transpose() << std::endl;
}

// ------------------------------------------------------------

TEST_F( ArticulationStateTest, SetState_RevoluteWithRotation_UpdatesJointStateCorrectly )
{
	Vec3d new_origin( 1, 0, 0 );
	Vec3d new_axis( 0, 1, 0 );

	Quaternion rotation = Quaternion::FromTwoVectors( Vec3d( 0, 0, 1 ), Vec3d( 0, 1, 0 ) );
	VecXd values = VecXd::Ones( 1 );

    auto revolute_state = Model::ArticulationState( revolute_articulation_ );
	revolute_state.SetState( rotation, new_origin, values );
    auto joint_states = revolute_state.GetJointStates();

    EXPECT_EQ( joint_states[0]->Value(), 1 );
    EXPECT_TRUE( joint_states[0]->Origin().isApprox( new_origin ) )
        << "Expected = " << new_origin.transpose() << std::endl
        << "Result   = " << joint_states[0]->Origin().transpose() << std::endl;
    EXPECT_TRUE( joint_states[0]->Axis().isApprox( new_axis ) )
        << "Expected = " << new_axis.transpose() << std::endl
        << "Result   = " << joint_states[0]->Axis().transpose() << std::endl;
}

// ------------------------------------------------------------

TEST_F( ArticulationStateTest, SetState_UniversalWithRotationAroundXAxis_UpdatesJointStateCorrectly )
{
	Vec3d new_origin( 1, 0, 0 );
    Vec3d new_axis( 0, 0, -1 );
	Quaternion rotation = Quaternion::FromTwoVectors( Vec3d( 0, 0, 1 ), Vec3d( 0, 1, 0 ) );
	VecXd values = VecXd::Zero( 2 );

    auto universal_state = Model::ArticulationState( universal_articulation_ );
	universal_state.SetState( rotation, new_origin, values );
    auto joint_states = universal_state.GetJointStates();

    EXPECT_EQ( joint_states[0]->Value(), 0 );
    EXPECT_TRUE( joint_states[0]->Origin().isApprox( Vec3d( 0.9, 0, 0 ) ) )
        << "Expected origin 0 = " << Vec3d( 0.9, 0, 0 ).transpose() << std::endl
        << "Result   origin 0 = " << joint_states[0]->Origin().transpose() << std::endl;
    EXPECT_TRUE( joint_states[0]->Axis().isApprox( Vec3d( 1, 0, 0 ) ) )
		<< "Expected axis 0 = " << Vec3d( 1, 0, 0 ).transpose() << std::endl
		<< "Result   axis 0 = " << joint_states[0]->Axis().transpose() << std::endl;

    EXPECT_EQ( joint_states[1]->Value(), 0 );
    EXPECT_TRUE( joint_states[1]->Origin().isApprox( Vec3d( 1, 0, -0.1 ) ) )
        << "Expected origin 1 = " << Vec3d( 1, 0, -0.1 ).transpose() << std::endl
        << "Result   origin 1 = " << joint_states[1]->Origin().transpose() << std::endl;
    EXPECT_TRUE( joint_states[1]->Axis().isApprox( Vec3d( 0, 0, -1 ) ) )
        << "Expected axis 1 = " << Vec3d( 0, 0, -1 ).transpose() << std::endl
        << "Result   axis 1 = " << joint_states[1]->Axis().transpose() << std::endl;
}

// ------------------------------------------------------------

TEST_F( ArticulationStateTest, SetState_UniversalWithRotationAroundYAxis_UpdatesJointStateCorrectly )
{
	Quaternion rotation = Quaternion::FromTwoVectors( Vec3d( 0, 0, 1 ), Vec3d( 1, 0, 0 ) );
	VecXd values = VecXd::Zero( 2 );

    auto universal_state = Model::ArticulationState( universal_articulation_ );
	universal_state.SetState( rotation, Vec3d( 1, 0, 0 ), values );
    auto joint_states = universal_state.GetJointStates();

    EXPECT_EQ( joint_states[0]->Value(), 0 );
    EXPECT_TRUE( joint_states[0]->Origin().isApprox( Vec3d( 1, 0, 0.1 ) ) )
        << "Expected origin 0 = " << Vec3d( 1, 0, 0.1 ).transpose() << std::endl
        << "Result   origin 0 = " << joint_states[0]->Origin().transpose() << std::endl;
    EXPECT_TRUE( joint_states[0]->Axis().isApprox( Vec3d( 0, 0, -1 ) ) )
		<< "Expected axis 0 = " << Vec3d( 0, 0, -1 ).transpose() << std::endl
		<< "Result   axis 0 = " << joint_states[0]->Axis().transpose() << std::endl;

    EXPECT_EQ( joint_states[1]->Value(), 0 );
    EXPECT_TRUE( joint_states[1]->Origin().isApprox( Vec3d( 1, 0.1, 0 ) ) )
        << "Expected origin 1 = " << Vec3d( 1, 0.1, 0 ).transpose() << std::endl
        << "Result   origin 1 = " << joint_states[1]->Origin().transpose() << std::endl;
    EXPECT_TRUE( joint_states[1]->Axis().isApprox( Vec3d( 0, 1, 0 ) ) )
        << "Expected axis 1 = " << Vec3d( 0, 1, 0 ).transpose() << std::endl
        << "Result   axis 1 = " << joint_states[1]->Axis().transpose() << std::endl;
}

// ------------------------------------------------------------

TEST_F( ArticulationStateTest, SetState_UniversalWithRotationAroundZAxis_UpdatesJointStateCorrectly )
{
	Quaternion rotation = Quaternion::FromTwoVectors( Vec3d( 0, 1, 0 ), Vec3d( 1, 0, 0 ) );
	VecXd values = VecXd::Zero( 2 );

    auto universal_state = Model::ArticulationState( universal_articulation_ );
	universal_state.SetState( rotation, Vec3d( 1, 0, 0 ), values );
    auto joint_states = universal_state.GetJointStates();

    EXPECT_EQ( joint_states[0]->Value(), 0 );
    EXPECT_TRUE( joint_states[0]->Origin().isApprox( Vec3d( 1, 0.1, 0 ) ) )
        << "Expected origin 0 = " << Vec3d( 1, 0.1, 0 ).transpose() << std::endl
        << "Result   origin 0 = " << joint_states[0]->Origin().transpose() << std::endl;
    EXPECT_TRUE( joint_states[0]->Axis().isApprox( Vec3d( 0, -1, 0 ) ) )
		<< "Expected axis 0 = " << Vec3d( 0, -1, 0 ).transpose() << std::endl
		<< "Result   axis 0 = " << joint_states[0]->Axis().transpose() << std::endl;

    EXPECT_EQ( joint_states[1]->Value(), 0 );
    EXPECT_TRUE( joint_states[1]->Origin().isApprox( Vec3d( 1.1, 0, 0 ) ) )
        << "Expected origin 1 = " << Vec3d( 1.1, 0, 0 ).transpose() << std::endl
        << "Result   origin 1 = " << joint_states[1]->Origin().transpose() << std::endl;
    EXPECT_TRUE( joint_states[1]->Axis().isApprox( Vec3d( 1, 0, 0 ) ) )
        << "Expected axis 1 = " << Vec3d( 1, 0, 0 ).transpose() << std::endl
        << "Result   axis 1 = " << joint_states[1]->Axis().transpose() << std::endl;
}

// ------------------------------------------------------------

TEST_F( ArticulationStateTest, SetState_SphericalWithRotationAroundXAxis_UpdatesJointStateCorrectly )
{
	Quaternion rotation = Quaternion::FromTwoVectors( Vec3d( 0, 0, 1 ), Vec3d( 0, 1, 0 ) );
	VecXd values = VecXd::Ones( 3 );

    auto spherical_state = Model::ArticulationState( spherical_articulation_ );
	spherical_state.SetState( rotation, Vec3d( 1, 0, 0 ), values );
    auto joint_states = spherical_state.GetJointStates();
        
    EXPECT_EQ( joint_states[0]->Value(), 1 );
    EXPECT_TRUE( joint_states[0]->Origin().isApprox( Vec3d( 1.1, 0, 0 ) ) )
        << "Expected = " << Vec3d( 1.1, 0, 0 ).transpose() << std::endl
        << "Result   = " << joint_states[0]->Origin().transpose() << std::endl;
    EXPECT_TRUE( joint_states[0]->Axis().isApprox( Vec3d( 1, 0, 0 ) ) )
        << "Expected = " << Vec3d( 1, 0, 0 ).transpose() << std::endl
        << "Result   = " << joint_states[0]->Axis().transpose() << std::endl;

    EXPECT_EQ( joint_states[1]->Value(), 1 );
    EXPECT_TRUE( joint_states[1]->Origin().isApprox( Vec3d( 1, 0, 0 ) ) )
        << "Expected = " << Vec3d( 1, 0, 0 ).transpose() << std::endl
        << "Result   = " << joint_states[1]->Origin().transpose() << std::endl;
    EXPECT_TRUE( joint_states[1]->Axis().isApprox( Vec3d( 0, 0, -1 ) ) )
        << "Expected = " << Vec3d( 0, 0, -1 ).transpose() << std::endl
        << "Result   = " << joint_states[1]->Axis().transpose() << std::endl;

    EXPECT_EQ( joint_states[2]->Value(), 1 );
    EXPECT_TRUE( joint_states[2]->Origin().isApprox( Vec3d( 1, 0.1, 0 ) ) )
        << "Expected = " << Vec3d( 1, 0.1, 0 ).transpose() << std::endl
        << "Result   = " << joint_states[2]->Origin().transpose() << std::endl;
    EXPECT_TRUE( joint_states[2]->Axis().isApprox( Vec3d( 0, 1, 0 ) ) )
        << "Expected = " << Vec3d( 0, 1, 0 ).transpose() << std::endl
        << "Result   = " << joint_states[2]->Axis().transpose() << std::endl;
}

// ============================================================
// SetPose
// ============================================================

TEST_F( ArticulationStateTest, SetPose_UpdatesOriginCorrectly )
{
	Vec3d new_origin( 2, 3, 4 );
	Quaternion rotation = Quaternion::Identity();

    auto revolute_state = Model::ArticulationState( revolute_articulation_ );
	revolute_state.SetCenterPose( rotation, new_origin );

	EXPECT_TRUE( revolute_state.Origin().isApprox( new_origin ) )
	    << "Origin should be updated by SetPose";
}

// ============================================================
// ApplyConstraints - Revolute
// ============================================================

TEST_F( ArticulationStateTest, ApplyConstraints_RevoluteWithValidDirection_UpdatesBoneDirection )
{
	// Setup revolute state with rotation and origin
	Vec3d origin( 0, 0, 0 );
	Quaternion rotation = Quaternion::Identity();
	VecXd values = VecXd::Zero( 1 );

    auto revolute_state = Model::ArticulationState( revolute_articulation_ );
	revolute_state.SetState( rotation, origin, values );

	// Create a bone and bone state
	auto bone = std::make_shared< const Model::Bone >( Vec3d( 1, 0, 0 ), Vec3d( 1, 0, 0 ) );
	Model::BoneState bone_state( bone );

	// Set an initial direction
	bone_state.Origin() = Vec3d( 0, 0, 0 );
	bone_state.Direction() = Vec3d( 1, 1, 1 ).normalized();

	// Apply constraints
	revolute_state.ApplyConstraints( bone_state );

	// Direction should be along the axis (z-axis for revolute joint)
	Vec3d expected_direction = Vec3d( 1, 1, 0 ).normalized() * bone->Length();
	EXPECT_TRUE( bone_state.Direction().isApprox( expected_direction ) )
	    << "Expected Direction = " << expected_direction.transpose() << std::endl
		<< "Result Direction = " << bone_state.Direction().transpose() << std::endl;
}

// ============================================================
// ApplyConstraints - Prismatic
// ============================================================

TEST_F( ArticulationStateTest, ApplyConstraints_PrismaticWithDirection_ConstrainsAlongAxis )
{
	// Setup prismatic state
	Vec3d origin( 0, 0, 0 );
	Quaternion rotation = Quaternion::Identity();
	VecXd values = VecXd::Zero( 1 );

    auto prismatic_state = Model::ArticulationState( prismatic_articulation_ );
	prismatic_state.SetState( rotation, origin, values );

	// Create a bone and bone state
	auto bone = std::make_shared< const Model::Bone >( Vec3d( 0, 0, 0 ), Vec3d( 0, 0, 1 ) );
	Model::BoneState bone_state( bone );

	// Set a direction that should be constrained
	bone_state.Direction() = Vec3d( 1, 1, 1 );

	// Apply prismatic constraints
	prismatic_state.ApplyConstraints( bone_state );

	// Direction should only have z component (along axis)
	EXPECT_NEAR( bone_state.Direction()[0], 0.0, 1e-9 )
	    << "X component should be zero for prismatic joint along Z";
	EXPECT_NEAR( bone_state.Direction()[1], 0.0, 1e-9 )
	    << "Y component should be zero for prismatic joint along Z";
	EXPECT_NEAR( std::abs( bone_state.Direction()[2] ), std::abs( Vec3d::UnitZ().dot( Vec3d( 1, 1, 1 ) ) ), 1e-5 )
	    << "Z component should be the projection of the input onto the axis";
}

// ============================================================
// ApplyConstraints - Universal
// ============================================================

TEST_F( ArticulationStateTest, ApplyConstraints_UniversalArticulation_ProducesValidDirection )
{
	// Setup universal state
	Vec3d origin( 0, 0, 0 );
	Quaternion rotation = Quaternion::Identity();
	VecXd values = VecXd::Zero( 2 );

    auto universal_state = Model::ArticulationState( universal_articulation_ );
	universal_state.SetState( rotation, origin, values );

	// Create a bone and bone state
	auto bone = std::make_shared< const Model::Bone >( Vec3d( 0.1, 0, 0 ), Vec3d( 0.1, 0.1, 0.1 ) );
	Model::BoneState bone_state( bone );

	// Set an initial direction
	double alpha = 0;
	double beta  = M_PI / 4;

	Iso3d T03 = Iso3d::Identity();
	T03.translate( Vec3d( 0.1, 0, 0 ) );
	T03.rotate( AngleAxis( alpha, Vec3d::UnitX() ) );
	T03.translate( Vec3d( 0, 0.1, 0 ) );
	T03.rotate( AngleAxis( beta, Vec3d::UnitY() ) );
	T03.translate( Vec3d( 0, 0, 0.1 ) );

	Vec3d expected_bone_state = T03.translation();
	bone_state.Direction() = T03.translation();

	std::cout << T03.translation() << std::endl;

	// Apply universal constraints
	universal_state.ApplyConstraints( bone_state );

	// Direction should have a valid magnitude
	EXPECT_TRUE( bone_state.Direction().isApprox( expected_bone_state ) ) 
		<< "Expected = " << expected_bone_state.transpose() << std::endl
		<< "Result   = " << bone_state.Direction().transpose() << std::endl;
}

// ============================================================
// ApplyConstraints - Spherical
// ============================================================

TEST_F( ArticulationStateTest, ApplyConstraints_SphericalArticulation_ProducesValidDirection )
{
	// Setup spherical state
	Vec3d origin( 0, 0, 0 );
	Quaternion rotation = Quaternion::Identity();
	VecXd values = VecXd::Zero( 3 );

    auto spherical_state = Model::ArticulationState( spherical_articulation_ );
	spherical_state.SetState( rotation, origin, values );

	// Create a bone and bone state
	auto bone = std::make_shared< const Model::Bone >( Vec3d( 0, 0, 0 ), Vec3d( 1, 0, 0 ) );
	Model::BoneState bone_state( bone );

	// Set an initial direction
	bone_state.Direction() = Vec3d( 1, 1, 0 ).normalized();

	// Apply spherical constraints
	spherical_state.ApplyConstraints( bone_state );

	// Direction should have a valid magnitude
	EXPECT_GT( bone_state.Direction().norm(), 0.0 )
	    << "Direction magnitude should be greater than zero after spherical constraint";
}

// ============================================================
// UpdateValue - Revolute
// ============================================================

TEST_F( ArticulationStateTest, UpdateValue_RevoluteFromZeroToNinetyDegrees )
{
	// Create two states: old and new
	Vec3d origin( 0, 0, 0 );
	Quaternion rotation = Quaternion::Identity();
	VecXd old_values = VecXd::Zero( 1 );

	auto old_state = std::make_shared< Model::ArticulationState >( revolute_articulation_ );
	old_state->SetState( rotation, origin, old_values );

	auto new_state = std::make_shared< Model::ArticulationState >( revolute_articulation_ );
	new_state->SetState( rotation, origin, old_values );

	// Create bone states for old and new configurations
	auto bone = std::make_shared< const Model::Bone >( Vec3d( 1, 0, 0 ), Vec3d( 1, 0, 0 ) );
	Model::BoneState old_bone_state( bone );
	Model::BoneState new_bone_state( bone );

	old_bone_state.Direction() = Vec3d( 1, 0, 0 );
	new_bone_state.Direction() = Vec3d( 0, 1, 0 ).normalized(); // Rotated 90 degrees

	// Note: UpdateValue requires the old_state as a reference
	// This test verifies it doesn't crash with valid inputs
	new_state->UpdateValue( *old_state, old_bone_state, new_bone_state );
	EXPECT_NEAR( new_state->Value(), M_PI / 2, 1e-5 );
}

// ============================================================
// UpdateValue - Universal
// ============================================================

TEST_F( ArticulationStateTest, UpdateValue_UniversalFromZeroToNinetyDegrees )
{
	// Create two states: old and new
	Vec3d origin( 0, 0, 0 );
	Quaternion rotation = Quaternion::Identity();
	VecXd values = VecXd::Zero( 2 );

    auto old_state = Model::ArticulationState( universal_articulation_ );
	old_state.SetState( rotation, origin, values );
	auto new_state = std::make_shared< Model::ArticulationState >( universal_articulation_ );
	new_state->SetState( rotation, origin, values );

	// Create a bone and bone state
	auto bone = std::make_shared< const Model::Bone >( Vec3d( 0.1, 0, 0 ), Vec3d( 0.1, 0.1, 0.1 ) );
	Model::BoneState bone_state( bone );

	// Set an initial direction
	double alpha = M_PI / 2;// M_PI / 4;
	double beta  = 0; // M_PI / 4;

	Mat4d T01;
	T01 << 1, 0, 0, 0.1,
		   0, cos( alpha ), -sin( alpha ), 0,
		   0, sin( alpha ), cos( alpha ), 0,
		   0, 0, 0, 1;
	Mat4d T12;
	T12 <<  cos( beta ), 0, -sin( beta ), 0,
			0, 1, 0, 0.1,
			sin( beta ), 0, cos( beta ), 0.1,
			0, 0, 0, 1;
	Mat4d T02 = T01 * T12;

	Model::BoneState old_bone_state( bone );
	Model::BoneState new_bone_state( bone );

	new_bone_state.Direction() = Translation( T02 );

	new_state->UpdateValue( old_state, old_bone_state, new_bone_state );

	auto new_values = new_state->GetJointValues();
	EXPECT_NEAR( new_values[0], alpha, 1e-5 );
	EXPECT_NEAR( new_values[1], beta, 1e-5 );
	EXPECT_NEAR( new_state->Value(), beta, 1e-5 );
}

// ============================================================
// Value accessor
// ============================================================

TEST_F( ArticulationStateTest, Value_ReturnsDefaultValueAfterConstruction )
{
    auto revolute_state = Model::ArticulationState( revolute_articulation_ );
	EXPECT_DOUBLE_EQ( revolute_state.Value(), 0.0 )
	    << "Value should return 0 for a newly constructed articulation state";
}

// ============================================================
// Integration tests
// ============================================================

TEST_F( ArticulationStateTest, Integration_SetStateAndGetJointValues_Consistent )
{
	Vec3d origin( 1, 2, 3 );
	Quaternion rotation = Quaternion::Identity();
	VecXd input_values = VecXd::Zero( 1 );
	input_values[0] = 0.5;

    auto revolute_state = Model::ArticulationState( revolute_articulation_ );
	revolute_state.SetState( rotation, origin, input_values );
	auto retrieved_values = revolute_state.GetJointValues();

	EXPECT_DOUBLE_EQ( retrieved_values[0], input_values[0] )
	    << "Joint values should be retrievable after setting state";
}

// ============================================================

} // namespace SOArm100::Kinematics::Test
