#include "Model/Skeleton/RevoluteArticulationState.hpp"

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
// Revolute Articulation State
// ============================================================

class RevoluteArticulationStateTest : public KinematicTestBase
{
protected:
void SetUp() override
{
	// Revolute articulation (single joint)
	revolute_joint_ = MakeRevoluteJoint(
		Vec3d( 0, 0, 1 ),
		Vec3d( 1, 0, 0 ),
		-M_PI / 2,
		M_PI / 2 );
	std::vector< Model::JointConstPtr > revolute_joints = { revolute_joint_ };
	revolute_articulation_ = std::make_shared< const Model::Articulation >(
		Model::ArticulationType::Revolute,
		revolute_joints,
		Vec3d( 1, 0, 0 ) );

	bone_off_axis_ = std::make_shared< const Model::Bone >( Vec3d( 1, 0, 0 ), Vec3d( 1, 0, 0 ) );
	bone_on_axis_ = std::make_shared< const Model::Bone >( Vec3d( 1, 0, 0 ), Vec3d( 0, 0, 0.1 ) );
}

void TearDown() override {
}

Model::JointConstPtr revolute_joint_;
Model::ArticulationConstPtr revolute_articulation_;
Model::BoneConstPtr bone_on_axis_;
Model::BoneConstPtr bone_off_axis_;
};

// ============================================================
// Constructor
// ============================================================

TEST_F( RevoluteArticulationStateTest, Constructor_InitialOriginMatchesArticulationCenter )
{
	auto state = Model::RevoluteArticulationState( revolute_articulation_ );
	EXPECT_EQ( 1, state.GetJointStates().size() );
	EXPECT_EQ( revolute_joint_->Axis(), state.GetJointStates()[0]->Axis() );
	EXPECT_EQ( revolute_joint_->Origin(), state.GetJointStates()[0]->Origin() );

	EXPECT_EQ( Vec3d::Zero(), state.LocalTransform().translation() );
	EXPECT_EQ( Mat3d::Identity(), state.LocalTransform().rotation() );

	EXPECT_EQ( revolute_articulation_->Center(), state.GlobalTransform().translation() );
	EXPECT_EQ( Mat3d::Identity(), state.GlobalTransform().rotation() );
}

// ============================================================
// GetJointValues
// ============================================================

TEST_F( RevoluteArticulationStateTest, GetJointValues_ReturnsVectorOfDefaultValues )
{
	auto state = Model::RevoluteArticulationState( revolute_articulation_ );
	auto values = state.GetJointValues();
	EXPECT_EQ( values.size(), 1 )
	    << "Revolute articulation should have 1 joint";
	EXPECT_DOUBLE_EQ( values[0], 0.0 )
	    << "Default joint value should be 0";
}

// ============================================================
// SetState
// ============================================================

TEST_F( RevoluteArticulationStateTest, SetState_WithRotation_UpdatesPoseAndValueCorrectly )
{
	Quaternion rotation = Quaternion::FromTwoVectors( Vec3d( 0, 0, 1 ), Vec3d( 0, 1, 0 ) );
	Vec3d expected_axis   = Vec3d( 0, 1, 0 );
	Vec3d expected_origin = Vec3d( 0, 0, 0 );
	VecXd expected_values( 1 );
	expected_values[0] = 0;

	Iso3d world_transform = Iso3d::Identity();
	world_transform.translate( expected_origin );
	world_transform.rotate( rotation );

	auto state = Model::RevoluteArticulationState( revolute_articulation_ );
	state.SetState( world_transform, expected_values );

	EXPECT_EQ( expected_values, state.GetJointValues() );

	EXPECT_EQ( Vec3d::Zero(), state.LocalTransform().translation() );
	EXPECT_EQ( Mat3d::Identity(), state.LocalTransform().rotation() );

	EXPECT_EQ( expected_origin, state.GlobalTransform().translation() );
	EXPECT_EQ( rotation.matrix(), state.GlobalTransform().rotation() );

	EXPECT_EQ( expected_values[0], state.GetJointStates()[0]->Value() );
	EXPECT_TRUE( state.GetJointStates()[0]->Origin().isApprox( expected_origin ) );
	EXPECT_TRUE( state.GetJointStates()[0]->Axis().isApprox( expected_axis ) );
}

// ------------------------------------------------------------

TEST_F( RevoluteArticulationStateTest, SetState_WithTranslation_UpdatesPoseAndValueCorrectly )
{
	Quaternion rotation = Quaternion::Identity();
	Vec3d expected_axis   = Vec3d( 0, 0, 1 );
	Vec3d expected_origin = Vec3d( 1, 1, 1 );
	VecXd expected_values( 1 );
	expected_values[0] = 0;

	Iso3d world_transform = Iso3d::Identity();
	world_transform.translate( expected_origin );
	world_transform.rotate( rotation );

	auto state = Model::RevoluteArticulationState( revolute_articulation_ );
	state.SetState( world_transform, expected_values );

	EXPECT_EQ( expected_values, state.GetJointValues() );

	EXPECT_EQ( Vec3d::Zero(), state.LocalTransform().translation() );
	EXPECT_EQ( Mat3d::Identity(), state.LocalTransform().rotation() );

	EXPECT_EQ( expected_origin, state.GlobalTransform().translation() );
	EXPECT_EQ( rotation.matrix(), state.GlobalTransform().rotation() );

	EXPECT_EQ( expected_values[0], state.GetJointStates()[0]->Value() );
	EXPECT_TRUE( state.GetJointStates()[0]->Origin().isApprox( expected_origin ) );
	EXPECT_TRUE( state.GetJointStates()[0]->Axis().isApprox( expected_axis ) );
}

// ------------------------------------------------------------

TEST_F( RevoluteArticulationStateTest, SetState_WithValue_UpdatesPoseAndValueCorrectly )
{
	Quaternion rotation = Quaternion::Identity();
	Vec3d expected_axis   = Vec3d( 0, 0, 1 );
	Vec3d expected_origin = Vec3d( 0, 0, 0 );
	VecXd expected_values( 1 );
	expected_values[0] = M_PI / 4;

	Iso3d world_transform = Iso3d::Identity();
	world_transform.translate( expected_origin );
	world_transform.rotate( rotation );

	auto state = Model::RevoluteArticulationState( revolute_articulation_ );
	state.SetState( world_transform, expected_values );

	EXPECT_EQ( expected_values, state.GetJointValues() );

	auto expected_local_rotation = AngleAxis( expected_values[0], Vec3d::UnitZ() );
	EXPECT_EQ( Vec3d::Zero(), state.LocalTransform().translation() );
	EXPECT_TRUE( expected_local_rotation.matrix().isApprox( state.LocalTransform().rotation() ) );

	auto expected_global_rotation = rotation * expected_local_rotation;
	EXPECT_EQ( expected_origin, state.GlobalTransform().translation() );
	EXPECT_TRUE( expected_global_rotation.matrix().isApprox( state.GlobalTransform().rotation() ) );

	EXPECT_EQ( expected_values[0], state.GetJointStates()[0]->Value() );
	EXPECT_TRUE( state.GetJointStates()[0]->Origin().isApprox( expected_origin ) );
	EXPECT_TRUE( state.GetJointStates()[0]->Axis().isApprox( expected_axis ) );
}

// ------------------------------------------------------------

TEST_F( RevoluteArticulationStateTest, SetState_WithRotationTranslationValue_UpdatesPoseAndValueCorrectly )
{
	Quaternion rotation = Quaternion::FromTwoVectors( Vec3d( 0, 0, 1 ), Vec3d( 0, 1, 0 ) );
	Vec3d expected_axis   = Vec3d( 0, 1, 0 );
	Vec3d expected_origin = Vec3d( 1, 1, 1 );
	VecXd expected_values( 1 );
	expected_values[0] = -M_PI / 4;

	Iso3d world_transform = Iso3d::Identity();
	world_transform.translate( expected_origin );
	world_transform.rotate( rotation );

	auto state = Model::RevoluteArticulationState( revolute_articulation_ );
	state.SetState( world_transform, expected_values );

	EXPECT_EQ( expected_values, state.GetJointValues() );

	auto expected_local_rotation = AngleAxis( expected_values[0], Vec3d::UnitZ() );
	EXPECT_EQ( Vec3d::Zero(), state.LocalTransform().translation() );
	EXPECT_TRUE( expected_local_rotation.matrix().isApprox( state.LocalTransform().rotation() ) );

	auto expected_global_rotation = rotation * expected_local_rotation;
	EXPECT_EQ( expected_origin, state.GlobalTransform().translation() );
	EXPECT_TRUE( expected_global_rotation.matrix().isApprox( state.GlobalTransform().rotation() ) );

	EXPECT_EQ( expected_values[0], state.GetJointStates()[0]->Value() );
	EXPECT_TRUE( state.GetJointStates()[0]->Origin().isApprox( expected_origin ) );
	EXPECT_TRUE( state.GetJointStates()[0]->Axis().isApprox( expected_axis ) );
}

// ============================================================
// SetPose
// ============================================================

TEST_F( RevoluteArticulationStateTest, SetPose_UpdatesOriginCorrectly )
{
	Quaternion rotation = Quaternion::FromTwoVectors( Vec3d( 0, 0, 1 ), Vec3d( 0, 1, 0 ) );
	Vec3d expected_axis   = Vec3d( 0, 1, 0 );
	Vec3d expected_origin = Vec3d( 1, 1, 1 );

	auto state = Model::RevoluteArticulationState( revolute_articulation_ );
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

	EXPECT_TRUE( state.GetJointStates()[0]->Origin().isApprox( expected_origin ) );
	EXPECT_TRUE( state.GetJointStates()[0]->Axis().isApprox( expected_axis ) );
}

// ============================================================
// Apply Constraints
// ============================================================

TEST_F( RevoluteArticulationStateTest, ApplyConstraints_BoneOnAxisWithinLimits_ExpectedResult )
{
	auto state = Model::RevoluteArticulationState( revolute_articulation_ );

	auto bone_state = Model::BoneState( bone_on_axis_ );
	bone_state.Origin() = Vec3d( 1, 0, 0 );
	bone_state.Direction() = Vec3d( 0.5, 0.5, 0.5 );

	auto rotation = Quaternion::Identity();
	auto center = Vec3d::Zero();
	Iso3d world_transform = Iso3d::Identity();
	world_transform.translate( center );
	world_transform.rotate( rotation );
	state.SetCenterPose( world_transform );
	state.ApplyConstraints( bone_state );

	Vec3d expected_bone_origin = Vec3d( 1, 0, 0 );
	Vec3d expected_bone_direction = revolute_joint_->Axis() * bone_on_axis_->Length();

	EXPECT_TRUE( bone_state.Origin().isApprox( expected_bone_origin ) )
	    << "Expected Origin   = " << expected_bone_origin.transpose() << std::endl
	    << "Constraint Origin = " << bone_state.Origin().transpose() << std::endl;

	EXPECT_TRUE( bone_state.Direction().isApprox( expected_bone_direction ) )
	    << "Expected Direction   = " << expected_bone_direction.transpose() << std::endl
	    << "Constraint Direction = " << bone_state.Direction().transpose() << std::endl;
}

// ------------------------------------------------------------

TEST_F( RevoluteArticulationStateTest, ApplyConstraints_BoneOffAxisWithinLimits_ExpectedResult )
{
	auto state = Model::RevoluteArticulationState( revolute_articulation_ );

	auto bone_state = Model::BoneState( bone_off_axis_ );
	bone_state.Origin() = Vec3d( 1, 0, 0 );
	bone_state.Direction() = Vec3d( 0.5, 0.5, 0.5 );

	auto rotation = Quaternion::Identity();
	auto center = Vec3d::Zero();
	Iso3d world_transform = Iso3d::Identity();
	world_transform.translate( center );
	world_transform.rotate( rotation );
	state.SetCenterPose( world_transform );
	state.ApplyConstraints( bone_state );

	Vec3d expected_bone_origin = Vec3d( 1, 0, 0 );
	Vec3d expected_bone_direction =
		bone_off_axis_->Length() * Vec3d( 1, 1, 0 ).normalized();

	EXPECT_TRUE( bone_state.Origin().isApprox( expected_bone_origin ) )
	    << "Expected Origin   = " << expected_bone_origin.transpose() << std::endl
	    << "Constraint Origin = " << bone_state.Origin().transpose() << std::endl;

	EXPECT_TRUE( bone_state.Direction().isApprox( expected_bone_direction ) )
	    << "Expected Direction   = " << expected_bone_direction.transpose() << std::endl
	    << "Constraint Direction = " << bone_state.Direction().transpose() << std::endl;
}

// ------------------------------------------------------------

TEST_F( RevoluteArticulationStateTest, ApplyConstraints_BoneOffAxisAboveUpperLimit_ExpectedResult )
{
	auto state = Model::RevoluteArticulationState( revolute_articulation_ );

	auto bone_state = Model::BoneState( bone_off_axis_ );
	bone_state.Origin() = Vec3d( 1, 0, 0 );
	bone_state.Direction() = Vec3d( -0.5, 0.5, 0.5 );

	auto rotation = Quaternion::Identity();
	auto center = Vec3d::Zero();
	Iso3d world_transform = Iso3d::Identity();
	world_transform.translate( center );
	world_transform.rotate( rotation );
	state.SetCenterPose( world_transform );
	state.ApplyConstraints( bone_state );

	Vec3d expected_bone_origin = Vec3d( 1, 0, 0 );
	Vec3d expected_bone_direction =
		bone_off_axis_->Length() * Vec3d( 0, 1, 0 ).normalized();

	EXPECT_TRUE( bone_state.Origin().isApprox( expected_bone_origin ) )
	    << "Expected Origin   = " << expected_bone_origin.transpose() << std::endl
	    << "Constraint Origin = " << bone_state.Origin().transpose() << std::endl;

	EXPECT_TRUE( bone_state.Direction().isApprox( expected_bone_direction ) )
	    << "Expected Direction   = " << expected_bone_direction.transpose() << std::endl
	    << "Constraint Direction = " << bone_state.Direction().transpose() << std::endl;
}

// ------------------------------------------------------------

TEST_F( RevoluteArticulationStateTest, ApplyConstraints_BoneOffAxisBelowLowerLimit_ExpectedResult )
{
	auto state = Model::RevoluteArticulationState( revolute_articulation_ );

	auto bone_state = Model::BoneState( bone_off_axis_ );
	bone_state.Origin() = Vec3d::Zero();
	bone_state.Direction() = Vec3d( -0.5, -0.5, 0.5 ).normalized();

	auto rotation = Quaternion::Identity();
	auto center = Vec3d::Zero();
	Iso3d world_transform = Iso3d::Identity();
	world_transform.translate( center );
	world_transform.rotate( rotation );
	state.SetCenterPose( world_transform );
	state.ApplyConstraints( bone_state );

	Vec3d expected_bone_origin = Vec3d::Zero();
	Vec3d expected_bone_direction =
		bone_off_axis_->Length() * Vec3d( 0, -1, 0 );

	EXPECT_TRUE( bone_state.Origin().isApprox( expected_bone_origin ) )
	    << "Expected Origin   = " << expected_bone_origin.transpose() << std::endl
	    << "Constraint Origin = " << bone_state.Origin().transpose() << std::endl;

	EXPECT_TRUE( bone_state.Direction().isApprox( expected_bone_direction ) )
	    << "Expected Direction   = " << expected_bone_direction.transpose() << std::endl
	    << "Constraint Direction = " << bone_state.Direction().transpose() << std::endl;
}

// ------------------------------------------------------------

TEST_F( RevoluteArticulationStateTest, ApplyConstraints_BoneOffAxisWithRotationTranslation_ExpectedResult )
{
	auto state = Model::RevoluteArticulationState( revolute_articulation_ );

	auto bone_state = Model::BoneState( bone_off_axis_ );
	bone_state.Origin() = Vec3d( 0, 0, 0 );
	bone_state.Direction() = Vec3d( 0.5, 0.5, 0.5 );

	auto rotation = Quaternion::FromTwoVectors( Vec3d::UnitZ(), Vec3d::UnitY() );
	auto center = Vec3d( -1, 0, 0 );
	Iso3d world_transform = Iso3d::Identity();
	world_transform.translate( center );
	world_transform.rotate( rotation );
	state.SetCenterPose( world_transform );
	state.ApplyConstraints( bone_state );

	Vec3d expected_bone_origin = Vec3d( 0, 0, 0 );
	Vec3d expected_bone_direction = bone_off_axis_->Length() * Vec3d( 1, 0, 1 ).normalized();

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

TEST_F( RevoluteArticulationStateTest, UpdateValues_BoneOffAxisWithRotationTranslationWithinLimits_ExpectedResult )
{
	auto state = Model::RevoluteArticulationState( revolute_articulation_ );

	auto bone_state = Model::BoneState( bone_off_axis_ );
	bone_state.Origin() = Vec3d( 0, 0, 0 );
	bone_state.Direction() = Vec3d( 0.5, 0.5, 0.5 );

	auto rotation = Quaternion::FromTwoVectors( Vec3d::UnitZ(), Vec3d::UnitY() );
	auto center = Vec3d( -1, 0, 0 );
	Iso3d world_transform = Iso3d::Identity();
	world_transform.translate( center );
	world_transform.rotate( rotation );
	state.SetCenterPose( world_transform );

	state.UpdateValues( bone_state );

	double expected_value = -M_PI / 4;
	EXPECT_EQ( state.GetJointValues().size(), 1 );
	EXPECT_NEAR( state.GetJointValues()[0], expected_value, epsilon );

	auto expected_local_rotation = AngleAxis( expected_value, revolute_joint_->Axis() );
	EXPECT_EQ( Vec3d( 0, 0, 0 ), state.LocalTransform().translation() );
	EXPECT_TRUE( expected_local_rotation.matrix().isApprox( state.LocalTransform().rotation() ) );

	EXPECT_EQ( center, state.GlobalTransform().translation() );
	EXPECT_TRUE( ( rotation * expected_local_rotation ).matrix().isApprox( state.GlobalTransform().rotation() ) );

	EXPECT_EQ( state.GetJointStates()[0]->Value(), expected_value );
	EXPECT_TRUE( state.GetJointStates()[0]->Origin().isApprox( center ) );
	EXPECT_TRUE( state.GetJointStates()[0]->Axis().isApprox( Vec3d( 0, 1, 0 ) ) );
}

// ------------------------------------------------------------

TEST_F( RevoluteArticulationStateTest, UpdateValues_BoneOffAxisWithRotationTranslationOutsideLimits_ExpectedResult )
{
	auto state = Model::RevoluteArticulationState( revolute_articulation_ );

	auto bone_state = Model::BoneState( bone_off_axis_ );
	bone_state.Origin() = Vec3d( 0, 0, 0 );
	bone_state.Direction() = Vec3d( -0.1, 0, 0.5 );

	auto rotation = Quaternion::FromTwoVectors( Vec3d::UnitZ(), Vec3d::UnitY() );
	auto center = Vec3d( -1, 0, 0 );
	Iso3d world_transform = Iso3d::Identity();
	world_transform.translate( center );
	world_transform.rotate( rotation );
	state.SetCenterPose( world_transform );

	state.UpdateValues( bone_state );

	double expected_value = -M_PI / 2;
	EXPECT_EQ( state.GetJointValues().size(), 1 );
	EXPECT_NEAR( state.GetJointValues()[0], expected_value, epsilon );

	auto expected_local_rotation = AngleAxis( expected_value, revolute_joint_->Axis() );
	EXPECT_EQ( Vec3d( 0, 0, 0 ), state.LocalTransform().translation() );
	EXPECT_TRUE( expected_local_rotation.matrix().isApprox( state.LocalTransform().rotation() ) );

	EXPECT_EQ( center, state.GlobalTransform().translation() );
	EXPECT_TRUE( ( rotation * expected_local_rotation ).matrix().isApprox( state.GlobalTransform().rotation() ) );

	EXPECT_EQ( state.GetJointStates()[0]->Value(), expected_value );
	EXPECT_TRUE( state.GetJointStates()[0]->Origin().isApprox( center ) );
	EXPECT_TRUE( state.GetJointStates()[0]->Axis().isApprox( Vec3d( 0, 1, 0 ) ) );
}

// ------------------------------------------------------------

TEST_F( RevoluteArticulationStateTest, UpdateValues_BoneOnAxisWithRotationTranslation_ExpectedResult )
{
	auto state = Model::RevoluteArticulationState( revolute_articulation_ );

	auto bone_state = Model::BoneState( bone_on_axis_ );
	bone_state.Origin() = Vec3d( 0, 0, 0 );
	bone_state.Direction() = Vec3d( 0, 0, 0.5 );

	auto rotation = Quaternion::FromTwoVectors( Vec3d::UnitZ(), Vec3d::UnitY() );
	auto center = Vec3d( -1, 0, 0 );
	Iso3d world_transform = Iso3d::Identity();
	world_transform.translate( center );
	world_transform.rotate( rotation );
	state.SetCenterPose( world_transform );

	state.UpdateValues( bone_state );

	double expected_value = 0;
	EXPECT_EQ( state.GetJointValues().size(), 1 );
	EXPECT_NEAR( state.GetJointValues()[0], expected_value, epsilon );

	auto expected_local_rotation = AngleAxis( expected_value, revolute_joint_->Axis() );
	EXPECT_EQ( Vec3d( 0, 0, 0 ), state.LocalTransform().translation() );
	EXPECT_TRUE( expected_local_rotation.matrix().isApprox( state.LocalTransform().rotation() ) );

	EXPECT_EQ( center, state.GlobalTransform().translation() );
	EXPECT_TRUE( ( rotation * expected_local_rotation ).matrix().isApprox( state.GlobalTransform().rotation() ) );

	EXPECT_EQ( state.GetJointStates()[0]->Value(), expected_value );
	EXPECT_TRUE( state.GetJointStates()[0]->Origin().isApprox( center ) );
	EXPECT_TRUE( state.GetJointStates()[0]->Axis().isApprox( Vec3d( 0, 1, 0 ) ) );
}

// ------------------------------------------------------------

} // namespace SOArm100::Kinematics::Test