#include "Model/Skeleton/UniversalArticulationState.hpp"

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
// Universal Articulation State
// ============================================================

class UniversalArticulationStateTest : public KinematicTestBase
{
protected:
void SetUp() override
{
	// Universal articulation (two joints)
	revolute_joint_1_ = MakeRevoluteJoint(
		Vec3d( 1, 0, 0 ),
		Vec3d( 0, 0, 0 ),
		-M_PI / 2,
		M_PI / 2 );
	revolute_joint_2_ = MakeRevoluteJoint(
		Vec3d( 0, 1, 0 ),
		Vec3d( 0.1, 0.1, 0 ),
		-M_PI / 2,
		M_PI / 2 );
	std::vector< Model::JointConstPtr > universal_joints = { revolute_joint_1_, revolute_joint_2_ };
	universal_articulation_ = std::make_shared< const Model::Articulation >(
		Model::ArticulationType::Universal,
		universal_joints,
		Vec3d( 0.1, 0, 0 ) );

	bone_off_axis_ = std::make_shared< const Model::Bone >( Vec3d( 0.1, 0, 0 ), Vec3d( 0, 0.1, 1 ) );
	bone_on_axis_ = std::make_shared< const Model::Bone >( Vec3d( 0.1, 0, 0 ), Vec3d( 0, 1, 0 ) );
}

void TearDown() override {
}

Vec3d BoneOffAxisInternalDirection( double angle1, double angle2 )
{
	Iso3d T03 = Iso3d::Identity();
	T03.translate( Vec3d( 0.1, 0, 0 ) );
	T03.rotate( AngleAxis( angle1, Vec3d::UnitX() ) );
	T03.translate( Vec3d( 0, 0.1, 0 ) );
	T03.rotate( AngleAxis( angle2, Vec3d::UnitY() ) );
	T03.translate( Vec3d( 0, 0, 1 ) );

	return T03.translation() - bone_off_axis_->Origin();
}

Model::JointConstPtr revolute_joint_1_;
Model::JointConstPtr revolute_joint_2_;
Model::ArticulationConstPtr universal_articulation_;
Model::BoneConstPtr bone_on_axis_;
Model::BoneConstPtr bone_off_axis_;
};

// ============================================================
// Constructor
// ============================================================

TEST_F( UniversalArticulationStateTest, Constructor_InitialOriginMatchesArticulationCenter )
{
	auto state = Model::UniversalArticulationState( universal_articulation_ );
	EXPECT_EQ( 2, state.GetJointStates().size() );

	EXPECT_EQ( revolute_joint_1_->Axis(), state.GetJointStates()[0]->Axis() );
	EXPECT_EQ( revolute_joint_1_->Origin(), state.GetJointStates()[0]->Origin() );

	EXPECT_EQ( revolute_joint_2_->Axis(), state.GetJointStates()[1]->Axis() );
	EXPECT_EQ( revolute_joint_2_->Origin(), state.GetJointStates()[1]->Origin() );

	EXPECT_EQ( Vec3d::Zero(), state.LocalTransform().translation() );
	EXPECT_EQ( Mat3d::Identity(), state.LocalTransform().rotation() );

	EXPECT_EQ( universal_articulation_->Center(), state.GlobalTransform().translation() );
	EXPECT_EQ( Mat3d::Identity(), state.GlobalTransform().rotation() );
}

// ============================================================
// GetJointValues
// ============================================================

TEST_F( UniversalArticulationStateTest, GetJointValues_ReturnsVectorOfDefaultValues )
{
	auto state = Model::UniversalArticulationState( universal_articulation_ );
	auto values = state.GetJointValues();
	EXPECT_EQ( values.size(), 2 )
	    << "Universal articulation should have 2 joint";
	EXPECT_DOUBLE_EQ( values[0], 0.0 );
	EXPECT_DOUBLE_EQ( values[1], 0.0 );
}

// ============================================================
// SetState
// ============================================================

TEST_F( UniversalArticulationStateTest, SetState_WithRotation_UpdatesPoseAndValueCorrectly )
{
	Quaternion rotation = Quaternion::FromTwoVectors( Vec3d( 0, 0, 1 ), Vec3d( 0, 1, 0 ) );
	Vec3d expected_origin = Vec3d( 0, 0, 0 );
	VecXd expected_values( 2 );
	expected_values << 0, 0;

	Iso3d world_transform = Iso3d::Identity();
	world_transform.translate( expected_origin );
	world_transform.rotate( rotation );

	auto state = Model::UniversalArticulationState( universal_articulation_ );
	state.SetState( world_transform, expected_values );

	EXPECT_EQ( expected_values, state.GetJointValues() );

	EXPECT_EQ( Vec3d::Zero(), state.LocalTransform().translation() );
	EXPECT_EQ( Mat3d::Identity(), state.LocalTransform().rotation() );

	EXPECT_EQ( expected_origin, state.GlobalTransform().translation() );
	EXPECT_EQ( rotation.matrix(), state.GlobalTransform().rotation() );

	EXPECT_EQ( expected_values[0], state.GetJointStates()[0]->Value() );
	EXPECT_TRUE( state.GetJointStates()[0]->Origin().isApprox(
					 Vec3d( -0.1, 0, 0 ) ) )
	    << "Expected Origin = " << std::endl << Vec3d( -0.1, 0, 0 ) << std::endl
	    << "Result Origin   = " << std::endl << state.GetJointStates()[0]->Origin() << std::endl;
	EXPECT_TRUE( state.GetJointStates()[0]->Axis().isApprox(
					 rotation * revolute_joint_1_->Axis() ) );

	auto local_rotation = AngleAxis( expected_values[0], revolute_joint_1_->Axis() );
	EXPECT_EQ( expected_values[1], state.GetJointStates()[1]->Value() );
	EXPECT_TRUE( state.GetJointStates()[1]->Origin().isApprox(
					 Vec3d( 0, 0, -0.1 ) ) )
	    << "Expected Origin = " << std::endl << Vec3d( 0, 0, -0.1 ) << std::endl
	    << "Result Origin   = " << std::endl << state.GetJointStates()[1]->Origin() << std::endl;
	EXPECT_TRUE( state.GetJointStates()[1]->Axis().isApprox(
					 rotation * local_rotation * revolute_joint_2_->Axis() ) );
}

// ------------------------------------------------------------

TEST_F( UniversalArticulationStateTest, SetState_WithTranslation_UpdatesPoseAndValueCorrectly )
{
	Quaternion rotation = Quaternion::Identity();
	Vec3d expected_origin = Vec3d( 1, 1, 1 );
	VecXd expected_values( 2 );
	expected_values << 0, 0;

	Iso3d world_transform = Iso3d::Identity();
	world_transform.translate( expected_origin );
	world_transform.rotate( rotation );

	auto state = Model::UniversalArticulationState( universal_articulation_ );
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

	auto local_rotation = AngleAxis( expected_values[0], revolute_joint_1_->Axis() );
	EXPECT_EQ( expected_values[1], state.GetJointStates()[1]->Value() );
	EXPECT_TRUE( state.GetJointStates()[1]->Origin().isApprox(
					 Vec3d( 1, 1.1, 1.0 ) ) )
	    << "Expected Origin = " << std::endl << Vec3d( 1, 1.1, 1.0 ) << std::endl
	    << "Result Origin   = " << std::endl << state.GetJointStates()[1]->Origin() << std::endl;
	EXPECT_TRUE( state.GetJointStates()[1]->Axis().isApprox(
					 rotation * local_rotation * revolute_joint_2_->Axis() ) );
}

// ------------------------------------------------------------

TEST_F( UniversalArticulationStateTest, SetState_WithValue_UpdatesPoseAndValueCorrectly )
{
	Quaternion rotation = Quaternion::Identity();
	Vec3d expected_origin = Vec3d( 0, 0, 0 );
	VecXd expected_values( 2 );
	expected_values << M_PI / 2, M_PI / 2;

	Iso3d world_transform = Iso3d::Identity();
	world_transform.translate( expected_origin );
	world_transform.rotate( rotation );

	auto state = Model::UniversalArticulationState( universal_articulation_ );
	state.SetState( world_transform, expected_values );

	EXPECT_EQ( expected_values, state.GetJointValues() );

	auto local_rotation_1 = AngleAxis( expected_values[0], revolute_joint_1_->Axis() );
	auto local_rotation_2 = AngleAxis( expected_values[1], revolute_joint_2_->Axis() );
	auto local_rotation = local_rotation_1 * local_rotation_2;
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
	EXPECT_TRUE( state.GetJointStates()[1]->Origin().isApprox(
					 Vec3d( 0, 0, 0.1 ) ) );
	EXPECT_TRUE( state.GetJointStates()[1]->Axis().isApprox(
					 rotation * local_rotation_1 * revolute_joint_2_->Axis() ) );
}

// ------------------------------------------------------------

TEST_F( UniversalArticulationStateTest, SetState_WithRotationTranslationValue_UpdatesPoseAndValueCorrectly )
{
	Quaternion rotation = Quaternion::FromTwoVectors( Vec3d( 0, 0, 1 ), Vec3d( 0, 1, 0 ) );
	Vec3d expected_origin = Vec3d( 1, 1, 1 );
	VecXd expected_values( 2 );
	expected_values << M_PI / 2, M_PI / 2;

	Iso3d world_transform = Iso3d::Identity();
	world_transform.translate( expected_origin );
	world_transform.rotate( rotation );

	auto state = Model::UniversalArticulationState( universal_articulation_ );
	state.SetState( world_transform, expected_values );

	EXPECT_EQ( expected_values, state.GetJointValues() );

	auto local_rotation_1 = AngleAxis( expected_values[0], revolute_joint_1_->Axis() );
	auto local_rotation_2 = AngleAxis( expected_values[1], revolute_joint_2_->Axis() );
	auto local_rotation = local_rotation_1 * local_rotation_2;
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
	EXPECT_TRUE( state.GetJointStates()[1]->Origin().isApprox(
					 Vec3d( 1.0, 1.1, 1.0 ) ) );
	EXPECT_TRUE( state.GetJointStates()[1]->Axis().isApprox(
					 rotation * local_rotation_1 * revolute_joint_2_->Axis() ) );
}

// ============================================================
// SetPose
// ============================================================

TEST_F( UniversalArticulationStateTest, SetPose_UpdatesOriginCorrectly )
{
	Quaternion rotation = Quaternion::FromTwoVectors( Vec3d( 0, 0, 1 ), Vec3d( 0, 1, 0 ) );
	Vec3d expected_origin = Vec3d( 1, 1, 1 );

	auto state = Model::UniversalArticulationState( universal_articulation_ );
	auto values = state.GetJointValues();

	Iso3d world_transform = Iso3d::Identity();
	world_transform.translate( expected_origin );
	world_transform.rotate( rotation );

	state.SetCenterPose( world_transform );

	EXPECT_EQ( values, state.GetJointValues() );

	EXPECT_EQ( Vec3d::Zero(), state.LocalTransform().translation() );
	EXPECT_EQ( Mat3d::Identity(), state.LocalTransform().rotation() );

	EXPECT_EQ( expected_origin, state.GlobalTransform().translation() );
	EXPECT_EQ( rotation.matrix(), state.GlobalTransform().rotation() );

	EXPECT_EQ( values[0], state.GetJointStates()[0]->Value() );
	EXPECT_TRUE( state.GetJointStates()[0]->Origin().isApprox(
					 Vec3d( 0.9, 1.0, 1.0 ) ) );
	EXPECT_TRUE( state.GetJointStates()[0]->Axis().isApprox(
					 rotation * revolute_joint_1_->Axis() ) );

	auto local_rotation = AngleAxis( values[0], revolute_joint_1_->Axis() );
	EXPECT_EQ( values[1], state.GetJointStates()[1]->Value() );
	EXPECT_TRUE( state.GetJointStates()[1]->Origin().isApprox(
					 Vec3d( 1.0, 1.0, 0.9 ) ) )
	    << "Expected = " << Vec3d( 1.0, 1.0, 0.9 ) << std::endl
	    << "Result   = " << state.GetJointStates()[1]->Origin() << std::endl;
	EXPECT_TRUE( state.GetJointStates()[1]->Axis().isApprox(
					 rotation * local_rotation * revolute_joint_2_->Axis() ) );
}

// ============================================================
// Apply Constraints
// ============================================================

TEST_F( UniversalArticulationStateTest, ApplyConstraints_BoneOffAxisFirstJointRotationWithinLimits_ExpectedResult )
{
	auto state = Model::UniversalArticulationState( universal_articulation_ );

	auto bone_state = Model::BoneState( bone_off_axis_ );

	Vec3d expected_bone_origin = Vec3d( 0.1, 0, 0 );
	Vec3d expected_bone_direction = BoneOffAxisInternalDirection( M_PI / 2, 0 );

	bone_state.Origin() = expected_bone_origin;
	bone_state.Direction() = expected_bone_direction;

	auto rotation = Quaternion::Identity();
	auto center = Vec3d::Zero();
	Iso3d world_transform = Iso3d::Identity();
	world_transform.translate( center );
	world_transform.rotate( rotation );
	state.SetCenterPose( world_transform );
	state.ApplyConstraints( bone_state );

	EXPECT_TRUE( bone_state.Origin().isApprox( expected_bone_origin ) )
	    << "Expected Origin   = " << expected_bone_origin.transpose() << std::endl
	    << "Constraint Origin = " << bone_state.Origin().transpose() << std::endl;

	EXPECT_TRUE( bone_state.Direction().isApprox( expected_bone_direction ) )
	    << "Expected Direction   = " << expected_bone_direction.transpose() << std::endl
	    << "Constraint Direction = " << bone_state.Direction().transpose() << std::endl;
}

// ------------------------------------------------------------

TEST_F( UniversalArticulationStateTest, ApplyConstraints_BoneOffAxisSecondJointRotationWithinLimits_ExpectedResult )
{
	auto state = Model::UniversalArticulationState( universal_articulation_ );

	auto bone_state = Model::BoneState( bone_off_axis_ );

	Vec3d expected_bone_origin = Vec3d( 0.1, 0, 0 );
	Vec3d expected_bone_direction = BoneOffAxisInternalDirection( 0, M_PI / 4 );

	bone_state.Origin() = expected_bone_origin;
	bone_state.Direction() = expected_bone_direction;

	auto rotation = Quaternion::Identity();
	auto center = Vec3d::Zero();
	Iso3d world_transform = Iso3d::Identity();
	world_transform.translate( center );
	world_transform.rotate( rotation );
	state.SetCenterPose( world_transform );
	state.ApplyConstraints( bone_state );

	EXPECT_TRUE( bone_state.Origin().isApprox( expected_bone_origin ) )
	    << "Expected Origin   = " << expected_bone_origin.transpose() << std::endl
	    << "Constraint Origin = " << bone_state.Origin().transpose() << std::endl;

	EXPECT_TRUE( bone_state.Direction().isApprox( expected_bone_direction ) )
	    << "Expected Direction   = " << expected_bone_direction.transpose() << std::endl
	    << "Constraint Direction = " << bone_state.Direction().transpose() << std::endl;
}

// ------------------------------------------------------------

TEST_F( UniversalArticulationStateTest, ApplyConstraints_BoneOffAxisCombinedRotationWithinLimits_ExpectedResult )
{
	auto state = Model::UniversalArticulationState( universal_articulation_ );

	auto bone_state = Model::BoneState( bone_off_axis_ );

	Vec3d expected_bone_origin = Vec3d( 0.1, 0, 0 );
	Vec3d expected_bone_direction = BoneOffAxisInternalDirection( M_PI / 4, M_PI / 4 );

	bone_state.Origin() = expected_bone_origin;
	bone_state.Direction() = expected_bone_direction;

	auto rotation = Quaternion::Identity();
	auto center = Vec3d::Zero();
	Iso3d world_transform = Iso3d::Identity();
	world_transform.translate( center );
	world_transform.rotate( rotation );
	state.SetCenterPose( world_transform );
	state.ApplyConstraints( bone_state );

	EXPECT_TRUE( bone_state.Origin().isApprox( expected_bone_origin ) )
	    << "Expected Origin   = " << expected_bone_origin.transpose() << std::endl
	    << "Constraint Origin = " << bone_state.Origin().transpose() << std::endl;

	EXPECT_TRUE( bone_state.Direction().isApprox( expected_bone_direction ) )
	    << "Expected Direction   = " << expected_bone_direction.transpose() << std::endl
	    << "Constraint Direction = " << bone_state.Direction().transpose() << std::endl;
}

// ------------------------------------------------------------

TEST_F( UniversalArticulationStateTest, ApplyConstraints_BoneOffAxisWithRotationTranslation_ExpectedResult )
{
	auto state = Model::UniversalArticulationState( universal_articulation_ );

	auto bone_state = Model::BoneState( bone_off_axis_ );
	auto rotation = Quaternion::FromTwoVectors( Vec3d::UnitZ(), Vec3d::UnitY() );
	auto center = Vec3d( 1, 1, 1 );

	Vec3d expected_bone_origin = center;
	Vec3d expected_bone_direction = rotation * BoneOffAxisInternalDirection( M_PI / 6, -M_PI / 4 );

	bone_state.Origin() = expected_bone_origin;
	bone_state.Direction() = expected_bone_direction;

	Iso3d world_transform = Iso3d::Identity();
	world_transform.translate( center );
	world_transform.rotate( rotation );
	state.SetCenterPose( world_transform );
	state.ApplyConstraints( bone_state );

	EXPECT_TRUE( bone_state.Origin().isApprox( expected_bone_origin ) )
	    << "Expected Origin   = " << expected_bone_origin.transpose() << std::endl
	    << "Constraint Origin = " << bone_state.Origin().transpose() << std::endl;

	EXPECT_TRUE( bone_state.Direction().isApprox( expected_bone_direction ) )
	    << "Expected Direction   = " << expected_bone_direction.transpose() << std::endl
	    << "Constraint Direction = " << bone_state.Direction().transpose() << std::endl;
}

// ------------------------------------------------------------

TEST_F( UniversalArticulationStateTest, ApplyConstraints_BoneOffAxisFirstJointRotationOutsideLimits_ExpectedResult )
{
	auto state = Model::UniversalArticulationState( universal_articulation_ );

	auto bone_state = Model::BoneState( bone_off_axis_ );

	Vec3d expected_bone_origin = Vec3d( 0.1, 0, 0 );
	Vec3d expected_bone_direction = BoneOffAxisInternalDirection( M_PI / 2, 0 );

	bone_state.Origin() = Vec3d( 0.1, 0, 0 );
	bone_state.Direction() = BoneOffAxisInternalDirection( 3 * M_PI / 4, 0 );

	auto rotation = Quaternion::Identity();
	auto center = Vec3d::Zero();
	Iso3d world_transform = Iso3d::Identity();
	world_transform.translate( center );
	world_transform.rotate( rotation );
	state.SetCenterPose( world_transform );
	state.ApplyConstraints( bone_state );

	EXPECT_TRUE( bone_state.Origin().isApprox( expected_bone_origin ) )
	    << "Expected Origin   = " << expected_bone_origin.transpose() << std::endl
	    << "Constraint Origin = " << bone_state.Origin().transpose() << std::endl;

	EXPECT_TRUE( bone_state.Direction().isApprox( expected_bone_direction ) )
	    << "Expected Direction   = " << expected_bone_direction.transpose() << std::endl
	    << "Constraint Direction = " << bone_state.Direction().transpose() << std::endl;
}

// ------------------------------------------------------------

TEST_F( UniversalArticulationStateTest, ApplyConstraints_BoneOffAxisSecondJointRotationOutsideLimits_ExpectedResult )
{
	auto state = Model::UniversalArticulationState( universal_articulation_ );

	auto bone_state = Model::BoneState( bone_off_axis_ );

	Vec3d expected_bone_origin = Vec3d( 0.1, 0, 0 );
	Vec3d expected_bone_direction = BoneOffAxisInternalDirection( 0, M_PI / 2 );

	bone_state.Origin() = Vec3d( 0.1, 0, 0 );
	bone_state.Direction() = BoneOffAxisInternalDirection( 0, 3 * M_PI / 4 );

	auto rotation = Quaternion::Identity();
	auto center = Vec3d::Zero();
	Iso3d world_transform = Iso3d::Identity();
	world_transform.translate( center );
	world_transform.rotate( rotation );
	state.SetCenterPose( world_transform );
	state.ApplyConstraints( bone_state );

	EXPECT_TRUE( bone_state.Origin().isApprox( expected_bone_origin ) )
	    << "Expected Origin   = " << expected_bone_origin.transpose() << std::endl
	    << "Constraint Origin = " << bone_state.Origin().transpose() << std::endl;

	EXPECT_TRUE( bone_state.Direction().isApprox( expected_bone_direction ) )
	    << "Expected Direction   = " << expected_bone_direction.transpose() << std::endl
	    << "Constraint Direction = " << bone_state.Direction().transpose() << std::endl;
}

// ------------------------------------------------------------

TEST_F( UniversalArticulationStateTest, ApplyConstraints_BoneOnAxisWithinLimits_ExpectedResult )
{
	auto state = Model::UniversalArticulationState( universal_articulation_ );

	auto bone_state = Model::BoneState( bone_on_axis_ );

	Vec3d expected_bone_origin = Vec3d( 0.1, 0, 0 );
	Vec3d expected_bone_direction = Vec3d( 0, 1, 1 ).normalized();

	bone_state.Origin() = expected_bone_origin;
	bone_state.Direction() = expected_bone_direction;

	auto rotation = Quaternion::Identity();
	auto center = Vec3d::Zero();
	Iso3d world_transform = Iso3d::Identity();
	world_transform.translate( center );
	world_transform.rotate( rotation );
	state.SetCenterPose( world_transform );
	state.ApplyConstraints( bone_state );

	EXPECT_TRUE( bone_state.Origin().isApprox( expected_bone_origin ) )
	    << "Expected Origin   = " << expected_bone_origin.transpose() << std::endl
	    << "Constraint Origin = " << bone_state.Origin().transpose() << std::endl;

	EXPECT_TRUE( bone_state.Direction().isApprox( expected_bone_direction ) )
	    << "Expected Direction   = " << expected_bone_direction.transpose() << std::endl
	    << "Constraint Direction = " << bone_state.Direction().transpose() << std::endl;
}

// ------------------------------------------------------------

TEST_F( UniversalArticulationStateTest, ApplyConstraints_BoneOnAxisWithRotationTranslation_ExpectedResult )
{
	auto state = Model::UniversalArticulationState( universal_articulation_ );

	auto bone_state = Model::BoneState( bone_on_axis_ );
	auto rotation = Quaternion::FromTwoVectors( Vec3d::UnitZ(), Vec3d::UnitX() );
	auto center = Vec3d( 1, 1, 1 );

	Vec3d expected_bone_origin = center;
	Vec3d expected_bone_direction = rotation * Vec3d( 0, 1, 1 ).normalized();

	bone_state.Origin() = expected_bone_origin;
	bone_state.Direction() = expected_bone_direction;

	Iso3d world_transform = Iso3d::Identity();
	world_transform.translate( center );
	world_transform.rotate( rotation );
	state.SetCenterPose( world_transform );
	state.ApplyConstraints( bone_state );

	EXPECT_TRUE( bone_state.Origin().isApprox( expected_bone_origin ) )
	    << "Expected Origin   = " << expected_bone_origin.transpose() << std::endl
	    << "Constraint Origin = " << bone_state.Origin().transpose() << std::endl;

	EXPECT_TRUE( bone_state.Direction().isApprox( expected_bone_direction ) )
	    << "Expected Direction   = " << expected_bone_direction.transpose() << std::endl
	    << "Constraint Direction = " << bone_state.Direction().transpose() << std::endl;
}

// ============================================================
// Update Value
// ============================================================

TEST_F( UniversalArticulationStateTest, UpdateValues_BoneOffAxisWithRotationTranslation_ExpectedResult )
{
	auto state = Model::UniversalArticulationState( universal_articulation_ );

	auto bone_state = Model::BoneState( bone_off_axis_ );
	auto rotation = Quaternion::FromTwoVectors( Vec3d::UnitZ(), Vec3d::UnitY() );
	auto center = Vec3d( 1, 1, 1 );

	double expected_value_1 = M_PI / 6;
	double expected_value_2 = -M_PI / 4;
	Vec3d bone_origin = center;
	Vec3d bone_direction = rotation * BoneOffAxisInternalDirection( expected_value_1, expected_value_2 );

	bone_state.Origin() = bone_origin;
	bone_state.Direction() = bone_direction;

	Iso3d world_transform = Iso3d::Identity();
	world_transform.translate( center );
	world_transform.rotate( rotation );
	state.SetCenterPose( world_transform );

	state.UpdateValues( bone_state );

	auto expected_local_rotation_1 = AngleAxis( expected_value_1, revolute_joint_1_->Axis() );
	auto expected_local_rotation_2 = AngleAxis( expected_value_2, revolute_joint_2_->Axis() );
	auto expected_local_rotation = expected_local_rotation_1 * expected_local_rotation_2;

	EXPECT_EQ( state.GetJointValues().size(), 2 );

	EXPECT_EQ( Vec3d( 0, 0, 0 ), state.LocalTransform().translation() );
	EXPECT_TRUE( expected_local_rotation.matrix().isApprox( state.LocalTransform().rotation() ) );

	EXPECT_EQ( center, state.GlobalTransform().translation() );
	EXPECT_TRUE( ( rotation * expected_local_rotation ).matrix().isApprox( state.GlobalTransform().rotation() ) );

	EXPECT_NEAR( state.GetJointValues()[0], expected_value_1, epsilon );
	EXPECT_NEAR( state.GetJointValues()[1], expected_value_2, epsilon );

	EXPECT_NEAR( state.GetJointStates()[0]->Value(), expected_value_1, epsilon );
	EXPECT_NEAR( state.GetJointStates()[1]->Value(), expected_value_2, epsilon );

	EXPECT_TRUE( state.GetJointStates()[0]->Origin().isApprox( Vec3d( 0.9, 1, 1 ) ) );
	EXPECT_TRUE( state.GetJointStates()[1]->Origin().isApprox( Vec3d( 1, 1 + 0.1 * sin( expected_value_1 ), 1 - 0.1 * cos( expected_value_1 ) ) ) )
	    << "Expected = " << Vec3d( 1, 1 + 0.1 * sin( expected_value_1 ), 1 - 0.1 * cos( expected_value_1 ) ) << std::endl
	    << "Result = " << state.GetJointStates()[1]->Origin() << std::endl;

	auto expected_axis_1 = rotation * revolute_joint_1_->Axis();
	auto expected_axis_2 = rotation * expected_local_rotation_1 * revolute_joint_2_->Axis();
	EXPECT_TRUE( state.GetJointStates()[0]->Axis().isApprox( expected_axis_1 ) );
	EXPECT_TRUE( state.GetJointStates()[1]->Axis().isApprox( expected_axis_2 ) );
}

// ------------------------------------------------------------

TEST_F( UniversalArticulationStateTest, UpdateValues_BoneOffAxisFirstJointRotationOutsideLimits_ExpectedResult )
{
	auto state = Model::UniversalArticulationState( universal_articulation_ );

	auto bone_state = Model::BoneState( bone_off_axis_ );

	double off_limit_value_1 = 3 * M_PI / 4;
	double expected_value_1 = M_PI / 2;
	double expected_value_2 = 0;
	bone_state.Origin() = Vec3d( 0.1, 0, 0 );
	bone_state.Direction() = BoneOffAxisInternalDirection( off_limit_value_1, expected_value_2 );

	auto rotation = Quaternion::Identity();
	auto center = Vec3d::Zero();
	Iso3d world_transform = Iso3d::Identity();
	world_transform.translate( center );
	world_transform.rotate( rotation );
	state.SetCenterPose( world_transform );

	state.UpdateValues( bone_state );

	auto expected_local_rotation_1 = AngleAxis( expected_value_1, revolute_joint_1_->Axis() );
	auto expected_local_rotation_2 = AngleAxis( expected_value_2, revolute_joint_2_->Axis() );
	auto expected_local_rotation = expected_local_rotation_1 * expected_local_rotation_2;

	EXPECT_EQ( state.GetJointValues().size(), 2 );

	EXPECT_EQ( Vec3d( 0, 0, 0 ), state.LocalTransform().translation() );
	EXPECT_TRUE( expected_local_rotation.matrix().isApprox( state.LocalTransform().rotation() ) );

	EXPECT_EQ( center, state.GlobalTransform().translation() );
	EXPECT_TRUE( ( rotation * expected_local_rotation ).matrix().isApprox( state.GlobalTransform().rotation() ) );

	EXPECT_NEAR( state.GetJointValues()[0], expected_value_1, epsilon );
	EXPECT_NEAR( state.GetJointValues()[1], expected_value_2, epsilon );

	EXPECT_NEAR( state.GetJointStates()[0]->Value(), expected_value_1, epsilon );
	EXPECT_NEAR( state.GetJointStates()[1]->Value(), expected_value_2, epsilon );

	EXPECT_TRUE( state.GetJointStates()[0]->Origin().isApprox( Vec3d( -0.1, 0, 0 ) ) );
	EXPECT_TRUE( state.GetJointStates()[1]->Origin().isApprox( Vec3d( 0, 0, 0.1 ) ) )
	    << "Expected = " <<  Vec3d( 0, 0, 0.1 ) << std::endl
	    << "Result = " << state.GetJointStates()[1]->Origin() << std::endl;

	auto expected_axis_1 = rotation * revolute_joint_1_->Axis();
	auto expected_axis_2 = rotation * expected_local_rotation_1 * revolute_joint_2_->Axis();
	EXPECT_TRUE( state.GetJointStates()[0]->Axis().isApprox( expected_axis_1 ) );
	EXPECT_TRUE( state.GetJointStates()[1]->Axis().isApprox( expected_axis_2 ) );
}

// ------------------------------------------------------------

TEST_F( UniversalArticulationStateTest, UpdateValues_BoneOffAxisSecondJointRotationOutsideLimits_ExpectedResult )
{
	auto state = Model::UniversalArticulationState( universal_articulation_ );

	auto bone_state = Model::BoneState( bone_off_axis_ );

	double off_limit_value_2 = 3 * M_PI / 4;
	double expected_value_1 = 0;
	double expected_value_2 = M_PI / 2;
	bone_state.Origin() = Vec3d( 0.1, 0, 0 );
	bone_state.Direction() = BoneOffAxisInternalDirection( expected_value_1, off_limit_value_2 );

	auto rotation = Quaternion::Identity();
	auto center = Vec3d::Zero();
	Iso3d world_transform = Iso3d::Identity();
	world_transform.translate( center );
	world_transform.rotate( rotation );
	state.SetCenterPose( world_transform );

	state.UpdateValues( bone_state );

	auto expected_local_rotation_1 = AngleAxis( expected_value_1, revolute_joint_1_->Axis() );
	auto expected_local_rotation_2 = AngleAxis( expected_value_2, revolute_joint_2_->Axis() );
	auto expected_local_rotation = expected_local_rotation_1 * expected_local_rotation_2;

	EXPECT_EQ( state.GetJointValues().size(), 2 );

	EXPECT_EQ( Vec3d( 0, 0, 0 ), state.LocalTransform().translation() );
	EXPECT_TRUE( expected_local_rotation.matrix().isApprox( state.LocalTransform().rotation() ) );

	EXPECT_EQ( center, state.GlobalTransform().translation() );
	EXPECT_TRUE( ( rotation * expected_local_rotation ).matrix().isApprox( state.GlobalTransform().rotation() ) );

	EXPECT_NEAR( state.GetJointValues()[0], expected_value_1, epsilon );
	EXPECT_NEAR( state.GetJointValues()[1], expected_value_2, epsilon );

	EXPECT_NEAR( state.GetJointStates()[0]->Value(), expected_value_1, epsilon );
	EXPECT_NEAR( state.GetJointStates()[1]->Value(), expected_value_2, epsilon );

	EXPECT_TRUE( state.GetJointStates()[0]->Origin().isApprox( Vec3d( -0.1, 0, 0 ) ) );
	EXPECT_TRUE( state.GetJointStates()[1]->Origin().isApprox( Vec3d( 0, 0.1, 0 ) ) )
	    << "Expected = " <<  Vec3d( 0, 0.1, 0 ) << std::endl
	    << "Result = " << state.GetJointStates()[1]->Origin() << std::endl;

	auto expected_axis_1 = rotation * revolute_joint_1_->Axis();
	auto expected_axis_2 = rotation * expected_local_rotation_1 * revolute_joint_2_->Axis();
	EXPECT_TRUE( state.GetJointStates()[0]->Axis().isApprox( expected_axis_1 ) );
	EXPECT_TRUE( state.GetJointStates()[1]->Axis().isApprox( expected_axis_2 ) );
}

// ------------------------------------------------------------

} // namespace SOArm100::Kinematics::Test
