#include "Model/Skeleton/SphericalArticulationState.hpp"

#include "Global.hpp"

#include "KinematicTestBase.hpp"
#include "Model/Joint/Joint.hpp"
#include "Model/Skeleton/Articulation.hpp"
#include "Model/Skeleton/ArticulationType.hpp"
#include "Model/Skeleton/Bone.hpp"
#include "Model/Skeleton/BoneState.hpp"
#include "Utils/KinematicsUtils.hpp"
#include "Utils/StringConverter.hpp"

#include <gtest/gtest.h>
#include <memory>
#include <cmath>
#include <ostream>

namespace SOArm100::Kinematics::Test
{

// ============================================================
// Spherical Articulation State
// ============================================================

class SphericalArticulationStateTest : public KinematicTestBase
{
protected:
void SetUp() override
{
	// Spherical articulation (two joints)
	revolute_joint_1_ = MakeRevoluteJoint(
		Vec3d( 1, 0, 0 ),
		Vec3d( 0, 0, 0 ),
		-M_PI,
		M_PI );
	revolute_joint_2_ = MakeRevoluteJoint(
		Vec3d( 0, 1, 0 ),
		Vec3d( 0.1, 0, 0 ),
		-M_PI / 2,
		M_PI / 2 );
    revolute_joint_3_ = MakeRevoluteJoint(
        Vec3d( 0, 0, 1 ),
        Vec3d( 0.1, 0, 0 ),
        -M_PI / 2,
        M_PI / 2 );
	std::vector< Model::JointConstPtr > spherical_joints = 
    { 
        revolute_joint_1_, 
        revolute_joint_2_,
        revolute_joint_3_
    };
	spherical_articulation_ = std::make_shared< const Model::Articulation >(
		Model::ArticulationType::Spherical,
		spherical_joints,
		Vec3d( 0.1, 0, 0 ) );

	bone_off_axis_ = std::make_shared< const Model::Bone >( Vec3d( 0.1, 0, 0 ), Vec3d( 1, 1, 1 ) );
	bone_on_axis_ = std::make_shared< const Model::Bone >( Vec3d( 0.1, 0, 0 ), Vec3d( 0, 0, 1 ) );
}

void TearDown() override {
}

Vec3d BoneOffAxisInternalDirection( 
    double angle1, 
    double angle2, 
    double angle3 )
{
	Iso3d T03 = Iso3d::Identity();
	T03.rotate( AngleAxis( angle1, Vec3d::UnitX() ) );
	T03.rotate( AngleAxis( angle2, Vec3d::UnitY() ) );
	T03.rotate( AngleAxis( angle3, Vec3d::UnitZ() ) );
	T03.translate( Vec3d( 1, 1, 1 ) );

	return T03.translation();
}

Model::JointConstPtr revolute_joint_1_;
Model::JointConstPtr revolute_joint_2_;
Model::JointConstPtr revolute_joint_3_;
Model::ArticulationConstPtr spherical_articulation_;
Model::BoneConstPtr bone_on_axis_;
Model::BoneConstPtr bone_off_axis_;
};

// ============================================================
// Constructor
// ============================================================

TEST_F( SphericalArticulationStateTest, Constructor_InitialOriginMatchesArticulationCenter )
{
	auto state = Model::SphericalArticulationState( spherical_articulation_ );
	EXPECT_EQ( 3, state.GetJointStates().size() );

	EXPECT_EQ( revolute_joint_1_->Axis(), state.GetJointStates()[0]->Axis() );
	EXPECT_EQ( revolute_joint_1_->Origin(), state.GetJointStates()[0]->Origin() );

	EXPECT_EQ( revolute_joint_2_->Axis(), state.GetJointStates()[1]->Axis() );
	EXPECT_EQ( revolute_joint_2_->Origin(), state.GetJointStates()[1]->Origin() );

	EXPECT_EQ( revolute_joint_3_->Axis(), state.GetJointStates()[2]->Axis() );
	EXPECT_EQ( revolute_joint_3_->Origin(), state.GetJointStates()[2]->Origin() );

	EXPECT_EQ( Vec3d::Zero(), state.LocalTransform().translation() );
	EXPECT_EQ( Mat3d::Identity(), state.LocalTransform().rotation() );

	EXPECT_EQ( spherical_articulation_->Center(), state.GlobalTransform().translation() );
	EXPECT_EQ( Mat3d::Identity(), state.GlobalTransform().rotation() );
}

// ============================================================
// GetJointValues
// ============================================================

TEST_F( SphericalArticulationStateTest, GetJointValues_ReturnsVectorOfDefaultValues )
{
	auto state = Model::SphericalArticulationState( spherical_articulation_ );
	auto values = state.GetJointValues();
	EXPECT_EQ( values.size(), 3 )
	    << "Spherical articulation should have 2 joint";
	EXPECT_DOUBLE_EQ( values[0], 0.0 );
	EXPECT_DOUBLE_EQ( values[1], 0.0 );
	EXPECT_DOUBLE_EQ( values[2], 0.0 );
}

// ============================================================
// SetState
// ============================================================

TEST_F( SphericalArticulationStateTest, SetState_WithRotation_UpdatesPoseAndValueCorrectly )
{
	Quaternion rotation = Quaternion::FromTwoVectors( Vec3d( 0, 0, 1 ), Vec3d( 0, 1, 0 ) );
	Vec3d expected_origin = Vec3d( 0, 0, 0 );
	VecXd expected_values( 3 );
	expected_values << 0, 0, 0;

	Iso3d world_transform = Iso3d::Identity();
	world_transform.translate( expected_origin );
	world_transform.rotate( rotation );

	auto state = Model::SphericalArticulationState( spherical_articulation_ );
	state.SetState( world_transform, expected_values );

	EXPECT_EQ( expected_values, state.GetJointValues() );

	EXPECT_EQ( Vec3d::Zero(), state.LocalTransform().translation() );
	EXPECT_EQ( Mat3d::Identity(), state.LocalTransform().rotation() );

	EXPECT_EQ( expected_origin, state.GlobalTransform().translation() );
	EXPECT_EQ( rotation.matrix(), state.GlobalTransform().rotation() );

	EXPECT_EQ( expected_values[0], state.GetJointStates()[0]->Value() );
	EXPECT_TRUE( state.GetJointStates()[0]->Origin().isApprox( Vec3d( -0.1, 0, 0 ) ) );
	EXPECT_TRUE( state.GetJointStates()[0]->Axis().isApprox(
					 rotation * revolute_joint_1_->Axis() ) );

	auto local_rotation_1 = AngleAxis( expected_values[0], revolute_joint_1_->Axis() );
	EXPECT_EQ( expected_values[1], state.GetJointStates()[1]->Value() );
	EXPECT_TRUE( state.GetJointStates()[1]->Origin().isApprox( expected_origin ) );
	EXPECT_TRUE( state.GetJointStates()[1]->Axis().isApprox(
					 rotation * local_rotation_1 * revolute_joint_2_->Axis() ) );

    auto local_rotation_2 = AngleAxis( expected_values[1], revolute_joint_2_->Axis() );
    EXPECT_EQ( expected_values[2], state.GetJointStates()[2]->Value() );
    EXPECT_TRUE( state.GetJointStates()[2]->Origin().isApprox( expected_origin ) );
    EXPECT_TRUE( state.GetJointStates()[2]->Axis().isApprox(
                    rotation * local_rotation_1 * local_rotation_2 * revolute_joint_3_->Axis() ) );
}

// ------------------------------------------------------------

TEST_F( SphericalArticulationStateTest, SetState_WithTranslation_UpdatesPoseAndValueCorrectly )
{
	Quaternion rotation = Quaternion::Identity();
	Vec3d expected_origin = Vec3d( 1, 1, 1 );
	VecXd expected_values( 3 );
	expected_values << 0, 0, 0;

	Iso3d world_transform = Iso3d::Identity();
	world_transform.translate( expected_origin );
	world_transform.rotate( rotation );

	auto state = Model::SphericalArticulationState( spherical_articulation_ );
	state.SetState( world_transform, expected_values );

	EXPECT_EQ( expected_values, state.GetJointValues() );

	EXPECT_EQ( Vec3d::Zero(), state.LocalTransform().translation() );
	EXPECT_EQ( Mat3d::Identity(), state.LocalTransform().rotation() );

	EXPECT_EQ( expected_origin, state.GlobalTransform().translation() );
	EXPECT_EQ( rotation.matrix(), state.GlobalTransform().rotation() );

	EXPECT_EQ( expected_values[0], state.GetJointStates()[0]->Value() );
	EXPECT_EQ( expected_values[0], state.GetJointStates()[0]->Value() );
	EXPECT_TRUE( state.GetJointStates()[0]->Origin().isApprox(
					 Vec3d( 0.9, 1, 1 ) ) )
	    << "Expected Origin = " << std::endl << Vec3d( 0.9, 1, 1 ) << std::endl
	    << "Result Origin   = " << std::endl << state.GetJointStates()[0]->Origin() << std::endl;
	EXPECT_TRUE( state.GetJointStates()[0]->Axis().isApprox(
					 rotation * revolute_joint_1_->Axis() ) );

    auto local_rotation_1 = AngleAxis( expected_values[0], revolute_joint_1_->Axis() );
    EXPECT_EQ( expected_values[1], state.GetJointStates()[1]->Value() );
    EXPECT_TRUE( state.GetJointStates()[1]->Origin().isApprox( expected_origin ) );
    EXPECT_TRUE( state.GetJointStates()[1]->Axis().isApprox(
                    rotation * local_rotation_1 * revolute_joint_2_->Axis() ) );

    auto local_rotation_2 = AngleAxis( expected_values[1], revolute_joint_2_->Axis() );
    EXPECT_EQ( expected_values[2], state.GetJointStates()[2]->Value() );
    EXPECT_TRUE( state.GetJointStates()[2]->Origin().isApprox( expected_origin ) );
    EXPECT_TRUE( state.GetJointStates()[2]->Axis().isApprox(
                    rotation * local_rotation_1 * local_rotation_2 * revolute_joint_3_->Axis() ) );
}

// ------------------------------------------------------------

TEST_F( SphericalArticulationStateTest, SetState_WithValue_UpdatesPoseAndValueCorrectly )
{
	Quaternion rotation = Quaternion::Identity();
	Vec3d expected_origin = Vec3d( 0, 0, 0 );
	VecXd expected_values( 3 );
	expected_values << M_PI / 2, M_PI / 2, M_PI / 2;

	Iso3d world_transform = Iso3d::Identity();
	world_transform.translate( expected_origin );
	world_transform.rotate( rotation );

	auto state = Model::SphericalArticulationState( spherical_articulation_ );
	state.SetState( world_transform, expected_values );

	EXPECT_EQ( expected_values, state.GetJointValues() );

	auto local_rotation_1 = AngleAxis( expected_values[0], revolute_joint_1_->Axis() );
	auto local_rotation_2 = AngleAxis( expected_values[1], revolute_joint_2_->Axis() );
	auto local_rotation_3 = AngleAxis( expected_values[2], revolute_joint_3_->Axis() );
	auto local_rotation = local_rotation_1 * local_rotation_2 * local_rotation_3;
	EXPECT_EQ( Vec3d::Zero(), state.LocalTransform().translation() );
	EXPECT_TRUE( local_rotation.matrix().isApprox( state.LocalTransform().rotation() ) );

	EXPECT_EQ( expected_origin, state.GlobalTransform().translation() );
	EXPECT_TRUE( local_rotation.matrix().isApprox( state.GlobalTransform().rotation() ) );

	EXPECT_EQ( expected_values[0], state.GetJointStates()[0]->Value() );
	EXPECT_TRUE( state.GetJointStates()[0]->Origin().isApprox(
					 Vec3d( -0.1, 0, 0 ) ) );
	EXPECT_TRUE( state.GetJointStates()[0]->Axis().isApprox(
					 rotation * revolute_joint_1_->Axis() ) );

    EXPECT_EQ( expected_values[1], state.GetJointStates()[1]->Value() );
    EXPECT_TRUE( state.GetJointStates()[1]->Origin().isApprox( expected_origin ) );
    EXPECT_TRUE( state.GetJointStates()[1]->Axis().isApprox(
                    rotation * local_rotation_1 * revolute_joint_2_->Axis() ) );

    EXPECT_EQ( expected_values[2], state.GetJointStates()[2]->Value() );
    EXPECT_TRUE( state.GetJointStates()[2]->Origin().isApprox( expected_origin ) );
    EXPECT_TRUE( state.GetJointStates()[2]->Axis().isApprox(
                    rotation * local_rotation_1 * local_rotation_2 * revolute_joint_3_->Axis() ) );
}

// ------------------------------------------------------------

TEST_F( SphericalArticulationStateTest, SetState_WithRotationTranslationValue_UpdatesPoseAndValueCorrectly )
{
	Quaternion rotation = Quaternion::FromTwoVectors( Vec3d( 0, 0, 1 ), Vec3d( 0, 1, 0 ) );
	Vec3d expected_origin = Vec3d( 1, 1, 1 );
	VecXd expected_values( 3 );
	expected_values << M_PI / 2, M_PI / 2, M_PI / 2;

	Iso3d world_transform = Iso3d::Identity();
	world_transform.translate( expected_origin );
	world_transform.rotate( rotation );

	auto state = Model::SphericalArticulationState( spherical_articulation_ );
	state.SetState( world_transform, expected_values );

	EXPECT_EQ( expected_values, state.GetJointValues() );

	auto local_rotation_1 = AngleAxis( expected_values[0], revolute_joint_1_->Axis() );
	auto local_rotation_2 = AngleAxis( expected_values[1], revolute_joint_2_->Axis() );
	auto local_rotation_3 = AngleAxis( expected_values[2], revolute_joint_3_->Axis() );
	auto local_rotation = local_rotation_1 * local_rotation_2 * local_rotation_3;
	EXPECT_EQ( Vec3d::Zero(), state.LocalTransform().translation() );
	EXPECT_TRUE( local_rotation.matrix().isApprox( state.LocalTransform().rotation() ) );

	EXPECT_EQ( expected_origin, state.GlobalTransform().translation() );
	EXPECT_TRUE( ( rotation * local_rotation ).matrix().isApprox( state.GlobalTransform().rotation() ) );

	EXPECT_EQ( expected_values[0], state.GetJointStates()[0]->Value() );
	EXPECT_TRUE( state.GetJointStates()[0]->Origin().isApprox(
					 Vec3d( 0.9, 1.0, 1.0 ) ) );
	EXPECT_TRUE( state.GetJointStates()[0]->Axis().isApprox(
					 rotation * revolute_joint_1_->Axis() ) );

    EXPECT_EQ( expected_values[1], state.GetJointStates()[1]->Value() );
    EXPECT_TRUE( state.GetJointStates()[1]->Origin().isApprox( expected_origin ) );
    EXPECT_TRUE( state.GetJointStates()[1]->Axis().isApprox(
                    rotation * local_rotation_1 * revolute_joint_2_->Axis() ) );

    EXPECT_EQ( expected_values[2], state.GetJointStates()[2]->Value() );
    EXPECT_TRUE( state.GetJointStates()[2]->Origin().isApprox( expected_origin ) );
    EXPECT_TRUE( state.GetJointStates()[2]->Axis().isApprox(
                    rotation * local_rotation_1 * local_rotation_2 * revolute_joint_3_->Axis() ) );
}

// ============================================================
// SetPose
// ============================================================

TEST_F( SphericalArticulationStateTest, SetPose_UpdatesOriginCorrectly )
{
	Quaternion rotation = Quaternion::FromTwoVectors( Vec3d( 0, 0, 1 ), Vec3d( 0, 1, 0 ) );
	Vec3d expected_origin = Vec3d( 1, 1, 1 );

	auto state = Model::SphericalArticulationState( spherical_articulation_ );
	auto values = state.GetJointValues();

	Iso3d world_transform = Iso3d::Identity();
	world_transform.translate( expected_origin );
	world_transform.rotate( rotation );

	state.SetCenterPose( world_transform );

	EXPECT_EQ( values, state.GetJointValues() );

	auto local_rotation_1 = AngleAxis( 0, revolute_joint_1_->Axis() );
	auto local_rotation_2 = AngleAxis( 0, revolute_joint_2_->Axis() );
	auto local_rotation_3 = AngleAxis( 0, revolute_joint_3_->Axis() );
	auto local_rotation = local_rotation_1 * local_rotation_2 * local_rotation_3;
	EXPECT_EQ( Vec3d::Zero(), state.LocalTransform().translation() );
	EXPECT_TRUE( local_rotation.matrix().isApprox( state.LocalTransform().rotation() ) );

	EXPECT_EQ( expected_origin, state.GlobalTransform().translation() );
	EXPECT_TRUE( ( rotation * local_rotation ).matrix().isApprox( state.GlobalTransform().rotation() ) );

	EXPECT_EQ( 0, state.GetJointStates()[0]->Value() );
	EXPECT_TRUE( state.GetJointStates()[0]->Origin().isApprox(
					 Vec3d( 0.9, 1.0, 1.0 ) ) );
	EXPECT_TRUE( state.GetJointStates()[0]->Axis().isApprox(
					 rotation * revolute_joint_1_->Axis() ) );

    EXPECT_EQ( 0, state.GetJointStates()[1]->Value() );
    EXPECT_TRUE( state.GetJointStates()[1]->Origin().isApprox( expected_origin ) );
    EXPECT_TRUE( state.GetJointStates()[1]->Axis().isApprox(
                    rotation * local_rotation_1 * revolute_joint_2_->Axis() ) );

    EXPECT_EQ( 0, state.GetJointStates()[2]->Value() );
    EXPECT_TRUE( state.GetJointStates()[2]->Origin().isApprox( expected_origin ) );
    EXPECT_TRUE( state.GetJointStates()[2]->Axis().isApprox(
                    rotation * local_rotation_1 * local_rotation_2 * revolute_joint_3_->Axis() ) );
}

// ============================================================
// Apply Constraints
// ============================================================



// ============================================================
// Update Value
// ============================================================

TEST_F( SphericalArticulationStateTest, UpdateValues_BoneOffAxisFirstJointRotationWithinLimits_ExpectedResult )
{
	auto state = Model::SphericalArticulationState( spherical_articulation_ );

	auto bone_state = Model::BoneState( bone_off_axis_ );

    auto rotation = Quaternion::Identity();
    auto center = Vec3d::Zero();

    double expected_value_1 = M_PI / 2;
    double expected_value_2 = 0;
    double expected_value_3 = 0;
	bone_state.Origin() = center;
	bone_state.Direction() = BoneOffAxisInternalDirection( 
        expected_value_1,
        expected_value_2, 
        expected_value_3 );

    Iso3d world_transform = Iso3d::Identity();
	world_transform.translate( center );
	world_transform.rotate( rotation );
	state.SetCenterPose( world_transform );

	state.UpdateValues( bone_state );

    auto result_direction = BoneOffAxisInternalDirection(
        state.GetJointValues()[0],
        state.GetJointValues()[1],
        state.GetJointValues()[2] );

    EXPECT_TRUE( result_direction.isApprox( bone_state.Direction() ) );

    auto expected_local_rotation_1 = AngleAxis( expected_value_1, revolute_joint_1_->Axis() );
	auto expected_local_rotation_2 = AngleAxis( expected_value_2, revolute_joint_2_->Axis() );
	auto expected_local_rotation_3 = AngleAxis( expected_value_3, revolute_joint_3_->Axis() );
	auto expected_local_rotation = expected_local_rotation_1 * expected_local_rotation_2 * expected_local_rotation_3;

	EXPECT_EQ( state.GetJointValues().size(), 3 );

	EXPECT_EQ( Vec3d( 0, 0, 0 ), state.LocalTransform().translation() );
	EXPECT_TRUE( expected_local_rotation.matrix().isApprox( state.LocalTransform().rotation() ) );

	EXPECT_EQ( center, state.GlobalTransform().translation() );
	EXPECT_TRUE( ( rotation * expected_local_rotation ).matrix().isApprox( state.GlobalTransform().rotation() ) );

	EXPECT_NEAR( state.GetJointValues()[0], expected_value_1, epsilon );
	EXPECT_NEAR( state.GetJointValues()[1], expected_value_2, epsilon );
	EXPECT_NEAR( state.GetJointValues()[2], expected_value_3, epsilon );

	EXPECT_NEAR( state.GetJointStates()[0]->Value(), expected_value_1, epsilon );
	EXPECT_NEAR( state.GetJointStates()[1]->Value(), expected_value_2, epsilon );
	EXPECT_NEAR( state.GetJointStates()[2]->Value(), expected_value_3, epsilon );

	EXPECT_TRUE( state.GetJointStates()[0]->Origin().isApprox( Vec3d( -0.1, 0, 0 ) ) );
	EXPECT_TRUE( state.GetJointStates()[1]->Origin().isApprox( center ) );
	EXPECT_TRUE( state.GetJointStates()[2]->Origin().isApprox( center ) );

	auto expected_axis_1 = rotation * revolute_joint_1_->Axis();
	auto expected_axis_2 = rotation * expected_local_rotation_1 * revolute_joint_2_->Axis();
	auto expected_axis_3 = rotation * expected_local_rotation_1 * expected_local_rotation_2 * revolute_joint_3_->Axis();
	EXPECT_TRUE( state.GetJointStates()[0]->Axis().isApprox( expected_axis_1 ) );
	EXPECT_TRUE( state.GetJointStates()[1]->Axis().isApprox( expected_axis_2 ) );
	EXPECT_TRUE( state.GetJointStates()[2]->Axis().isApprox( expected_axis_3 ) );
}

// ------------------------------------------------------------

TEST_F( SphericalArticulationStateTest, UpdateValues_BoneOffAxisSecondJointRotationWithinLimits_ExpectedResult )
{
	auto state = Model::SphericalArticulationState( spherical_articulation_ );

	auto bone_state = Model::BoneState( bone_off_axis_ );

	auto rotation = Quaternion::Identity();
	auto center = Vec3d::Zero();

    double expected_value_1 = 0;
    double expected_value_2 = M_PI / 4;
    double expected_value_3 = 0;
	bone_state.Origin() = center;
	bone_state.Direction() = BoneOffAxisInternalDirection( 
        expected_value_1,
        expected_value_2, 
        expected_value_3 );

    Iso3d world_transform = Iso3d::Identity();
	world_transform.translate( center );
	world_transform.rotate( rotation );
	state.SetCenterPose( world_transform );

	state.UpdateValues( bone_state );

    auto result_direction = BoneOffAxisInternalDirection(
        state.GetJointValues()[0],
        state.GetJointValues()[1],
        state.GetJointValues()[2] );

    EXPECT_TRUE( result_direction.isApprox( bone_state.Direction() ) );

    auto expected_local_rotation_1 = AngleAxis( expected_value_1, revolute_joint_1_->Axis() );
	auto expected_local_rotation_2 = AngleAxis( expected_value_2, revolute_joint_2_->Axis() );
	auto expected_local_rotation_3 = AngleAxis( expected_value_3, revolute_joint_3_->Axis() );
	auto expected_local_rotation = expected_local_rotation_1 * expected_local_rotation_2 * expected_local_rotation_3;

	EXPECT_EQ( state.GetJointValues().size(), 3 );

	EXPECT_EQ( Vec3d( 0, 0, 0 ), state.LocalTransform().translation() );
	EXPECT_TRUE( expected_local_rotation.matrix().isApprox( state.LocalTransform().rotation() ) );

	EXPECT_EQ( center, state.GlobalTransform().translation() );
	EXPECT_TRUE( ( rotation * expected_local_rotation ).matrix().isApprox( state.GlobalTransform().rotation() ) );

	EXPECT_NEAR( state.GetJointValues()[0], expected_value_1, epsilon );
	EXPECT_NEAR( state.GetJointValues()[1], expected_value_2, epsilon );
	EXPECT_NEAR( state.GetJointValues()[2], expected_value_3, epsilon );

	EXPECT_NEAR( state.GetJointStates()[0]->Value(), expected_value_1, epsilon );
	EXPECT_NEAR( state.GetJointStates()[1]->Value(), expected_value_2, epsilon );
	EXPECT_NEAR( state.GetJointStates()[2]->Value(), expected_value_3, epsilon );

	EXPECT_TRUE( state.GetJointStates()[0]->Origin().isApprox( Vec3d( -0.1, 0, 0 ) ) );
	EXPECT_TRUE( state.GetJointStates()[1]->Origin().isApprox( center ) );
	EXPECT_TRUE( state.GetJointStates()[2]->Origin().isApprox( center ) );

	auto expected_axis_1 = rotation * revolute_joint_1_->Axis();
	auto expected_axis_2 = rotation * expected_local_rotation_1 * revolute_joint_2_->Axis();
	auto expected_axis_3 = rotation * expected_local_rotation_1 * expected_local_rotation_2 * revolute_joint_3_->Axis();
	EXPECT_TRUE( state.GetJointStates()[0]->Axis().isApprox( expected_axis_1 ) );
	EXPECT_TRUE( state.GetJointStates()[1]->Axis().isApprox( expected_axis_2 ) );
	EXPECT_TRUE( state.GetJointStates()[2]->Axis().isApprox( expected_axis_3 ) );
}

// ------------------------------------------------------------

TEST_F( SphericalArticulationStateTest, UpdateValues_BoneOffAxisThirdJointRotationWithinLimits_ExpectedResult )
{
	auto state = Model::SphericalArticulationState( spherical_articulation_ );

	auto bone_state = Model::BoneState( bone_off_axis_ );

	auto rotation = Quaternion::Identity();
	auto center = Vec3d::Zero();

    double expected_value_1 = 0;
    double expected_value_2 = 0;
    double expected_value_3 = M_PI / 4;
	bone_state.Origin() = center;
	bone_state.Direction() = BoneOffAxisInternalDirection( 
        expected_value_1,
        expected_value_2, 
        expected_value_3 );

    Iso3d world_transform = Iso3d::Identity();
	world_transform.translate( center );
	world_transform.rotate( rotation );
	state.SetCenterPose( world_transform );

	state.UpdateValues( bone_state );

    auto expected_local_rotation_1 = AngleAxis( expected_value_1, revolute_joint_1_->Axis() );
	auto expected_local_rotation_2 = AngleAxis( expected_value_2, revolute_joint_2_->Axis() );
	auto expected_local_rotation_3 = AngleAxis( expected_value_3, revolute_joint_3_->Axis() );
	auto expected_local_rotation = expected_local_rotation_1 * expected_local_rotation_2 * expected_local_rotation_3;

	EXPECT_EQ( state.GetJointValues().size(), 3 );

	EXPECT_EQ( Vec3d( 0, 0, 0 ), state.LocalTransform().translation() );
	EXPECT_TRUE( expected_local_rotation.matrix().isApprox( state.LocalTransform().rotation() ) );

	EXPECT_EQ( center, state.GlobalTransform().translation() );
	EXPECT_TRUE( ( rotation * expected_local_rotation ).matrix().isApprox( state.GlobalTransform().rotation() ) );

	EXPECT_NEAR( state.GetJointValues()[0], expected_value_1, epsilon );
	EXPECT_NEAR( state.GetJointValues()[1], expected_value_2, epsilon );
	EXPECT_NEAR( state.GetJointValues()[2], expected_value_3, epsilon );

	EXPECT_NEAR( state.GetJointStates()[0]->Value(), expected_value_1, epsilon );
	EXPECT_NEAR( state.GetJointStates()[1]->Value(), expected_value_2, epsilon );
	EXPECT_NEAR( state.GetJointStates()[2]->Value(), expected_value_3, epsilon );

	EXPECT_TRUE( state.GetJointStates()[0]->Origin().isApprox( Vec3d( -0.1, 0, 0 ) ) );
	EXPECT_TRUE( state.GetJointStates()[1]->Origin().isApprox( center ) );
	EXPECT_TRUE( state.GetJointStates()[2]->Origin().isApprox( center ) );

	auto expected_axis_1 = rotation * revolute_joint_1_->Axis();
	auto expected_axis_2 = rotation * expected_local_rotation_1 * revolute_joint_2_->Axis();
	auto expected_axis_3 = rotation * expected_local_rotation_1 * expected_local_rotation_2 * revolute_joint_3_->Axis();
	EXPECT_TRUE( state.GetJointStates()[0]->Axis().isApprox( expected_axis_1 ) );
	EXPECT_TRUE( state.GetJointStates()[1]->Axis().isApprox( expected_axis_2 ) );
	EXPECT_TRUE( state.GetJointStates()[2]->Axis().isApprox( expected_axis_3 ) );
}

// ------------------------------------------------------------

TEST_F( SphericalArticulationStateTest, UpdateValues_BoneOffAxisCombinedRotationWithinLimits_ExpectedResult )
{
	auto state = Model::SphericalArticulationState( spherical_articulation_ );

	auto bone_state = Model::BoneState( bone_off_axis_ );

	auto rotation = Quaternion::Identity();
	auto center = Vec3d::Zero();

    double expected_value_1 = M_PI / 4;
    double expected_value_2 = M_PI / 4;
    double expected_value_3 = M_PI / 4;
	bone_state.Origin() = center;
	bone_state.Direction() = BoneOffAxisInternalDirection( 
        expected_value_1,
        expected_value_2, 
        expected_value_3 );

    Iso3d world_transform = Iso3d::Identity();
	world_transform.translate( center );
	world_transform.rotate( rotation );
	state.SetCenterPose( world_transform );

	state.UpdateValues( bone_state );

    auto expected_local_rotation_1 = AngleAxis( expected_value_1, revolute_joint_1_->Axis() );
	auto expected_local_rotation_2 = AngleAxis( expected_value_2, revolute_joint_2_->Axis() );
	auto expected_local_rotation_3 = AngleAxis( expected_value_3, revolute_joint_3_->Axis() );
	auto expected_local_rotation = expected_local_rotation_1 * expected_local_rotation_2 * expected_local_rotation_3;

	EXPECT_EQ( state.GetJointValues().size(), 3 );

	EXPECT_EQ( Vec3d( 0, 0, 0 ), state.LocalTransform().translation() );
	EXPECT_TRUE( expected_local_rotation.matrix().isApprox( state.LocalTransform().rotation() ) );

	EXPECT_EQ( center, state.GlobalTransform().translation() );
	EXPECT_TRUE( ( rotation * expected_local_rotation ).matrix().isApprox( state.GlobalTransform().rotation() ) );

	EXPECT_NEAR( state.GetJointValues()[0], expected_value_1, epsilon );
	EXPECT_NEAR( state.GetJointValues()[1], expected_value_2, epsilon );
	EXPECT_NEAR( state.GetJointValues()[2], expected_value_3, epsilon );

	EXPECT_NEAR( state.GetJointStates()[0]->Value(), expected_value_1, epsilon );
	EXPECT_NEAR( state.GetJointStates()[1]->Value(), expected_value_2, epsilon );
	EXPECT_NEAR( state.GetJointStates()[2]->Value(), expected_value_3, epsilon );

	EXPECT_TRUE( state.GetJointStates()[0]->Origin().isApprox( Vec3d( -0.1, 0, 0 ) ) );
	EXPECT_TRUE( state.GetJointStates()[1]->Origin().isApprox( center ) );
	EXPECT_TRUE( state.GetJointStates()[2]->Origin().isApprox( center ) );

	auto expected_axis_1 = rotation * revolute_joint_1_->Axis();
	auto expected_axis_2 = rotation * expected_local_rotation_1 * revolute_joint_2_->Axis();
	auto expected_axis_3 = rotation * expected_local_rotation_1 * expected_local_rotation_2 * revolute_joint_3_->Axis();
	EXPECT_TRUE( state.GetJointStates()[0]->Axis().isApprox( expected_axis_1 ) );
	EXPECT_TRUE( state.GetJointStates()[1]->Axis().isApprox( expected_axis_2 ) );
	EXPECT_TRUE( state.GetJointStates()[2]->Axis().isApprox( expected_axis_3 ) );
}

// ------------------------------------------------------------

TEST_F( SphericalArticulationStateTest, ApplyConstraints_BoneOffAxisWithRotationTranslation_ExpectedResult )
{
	auto state = Model::SphericalArticulationState( spherical_articulation_ );

	auto bone_state = Model::BoneState( bone_off_axis_ );
	auto rotation = Quaternion::FromTwoVectors( Vec3d::UnitZ(), Vec3d::UnitY() );
	auto center = Vec3d( 1, 1, 1 );
    
    double expected_value_1 = M_PI / 4;
    double expected_value_2 = -M_PI / 6;
    double expected_value_3 = M_PI / 3;
	bone_state.Origin() = center;
	bone_state.Direction() = BoneOffAxisInternalDirection( 
        expected_value_1,
        expected_value_2, 
        expected_value_3 );

    Iso3d world_transform = Iso3d::Identity();
	world_transform.translate( center );
	world_transform.rotate( rotation );
	state.SetCenterPose( world_transform );

	state.UpdateValues( bone_state );

    auto expected_local_rotation_1 = AngleAxis( expected_value_1, revolute_joint_1_->Axis() );
	auto expected_local_rotation_2 = AngleAxis( expected_value_2, revolute_joint_2_->Axis() );
	auto expected_local_rotation_3 = AngleAxis( expected_value_3, revolute_joint_3_->Axis() );
	auto expected_local_rotation = expected_local_rotation_1 * expected_local_rotation_2 * expected_local_rotation_3;

	EXPECT_EQ( state.GetJointValues().size(), 3 );

	EXPECT_EQ( Vec3d( 0, 0, 0 ), state.LocalTransform().translation() );
	EXPECT_TRUE( expected_local_rotation.matrix().isApprox( state.LocalTransform().rotation() ) );

	EXPECT_EQ( center, state.GlobalTransform().translation() );
	EXPECT_TRUE( ( rotation * expected_local_rotation ).matrix().isApprox( state.GlobalTransform().rotation() ) );

	EXPECT_NEAR( state.GetJointValues()[0], expected_value_1, epsilon );
	EXPECT_NEAR( state.GetJointValues()[1], expected_value_2, epsilon );
	EXPECT_NEAR( state.GetJointValues()[2], expected_value_3, epsilon );

	EXPECT_NEAR( state.GetJointStates()[0]->Value(), expected_value_1, epsilon );
	EXPECT_NEAR( state.GetJointStates()[1]->Value(), expected_value_2, epsilon );
	EXPECT_NEAR( state.GetJointStates()[2]->Value(), expected_value_3, epsilon );

	EXPECT_TRUE( state.GetJointStates()[0]->Origin().isApprox( Vec3d( -0.1, 0, 0 ) ) );
	EXPECT_TRUE( state.GetJointStates()[1]->Origin().isApprox( center ) );
	EXPECT_TRUE( state.GetJointStates()[2]->Origin().isApprox( center ) );

	auto expected_axis_1 = rotation * revolute_joint_1_->Axis();
	auto expected_axis_2 = rotation * expected_local_rotation_1 * revolute_joint_2_->Axis();
	auto expected_axis_3 = rotation * expected_local_rotation_1 * expected_local_rotation_2 * revolute_joint_3_->Axis();
	EXPECT_TRUE( state.GetJointStates()[0]->Axis().isApprox( expected_axis_1 ) );
	EXPECT_TRUE( state.GetJointStates()[1]->Axis().isApprox( expected_axis_2 ) );
	EXPECT_TRUE( state.GetJointStates()[2]->Axis().isApprox( expected_axis_3 ) );
}

// ------------------------------------------------------------



// ------------------------------------------------------------

} // namespace SOArm100::Kinematics::Test
