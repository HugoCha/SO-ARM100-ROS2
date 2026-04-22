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
	auto joint1 = MakeRevoluteJoint(
		Vec3d( 1, 0, 0 ),
		Vec3d( 0, 0, 0 ),
		-M_PI / 2,
		M_PI / 2 );
	auto joint2 = MakeRevoluteJoint(
		Vec3d( 0, 1, 0 ),
		Vec3d( 0.1, 0.1, 0 ),
		-M_PI / 2,
		M_PI / 2 );
	std::vector< Model::JointConstPtr > universal_joints = { joint1, joint2 };
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
	EXPECT_TRUE( state.Origin().isApprox( universal_articulation_->Center() ) )
	    << "Initial origin should match the articulation's center";
}

// ------------------------------------------------------------

TEST_F( UniversalArticulationStateTest, Constructor_InitialAxisMatchesLastJointAxis )
{
	// For revolute, the axis should be the revolute joint's axis
	auto state = Model::UniversalArticulationState( universal_articulation_ );
	EXPECT_TRUE( state.Axis().isApprox( universal_articulation_->Axis() ) )
	    << "Axis should match the last joint's axis";
}

// ------------------------------------------------------------

TEST_F( UniversalArticulationStateTest, Constructor_GetArticulationReturnsCorrectArticulation )
{
	auto state = Model::UniversalArticulationState( universal_articulation_ );
	EXPECT_EQ( state.GetArticulation(), universal_articulation_ )
	    << "GetArticulation should return the same articulation passed to constructor";
}

// ============================================================
// GetJointValues
// ============================================================

TEST_F( UniversalArticulationStateTest, GetJointValues_ReturnsVectorOfDefaultValues )
{
	auto state = Model::UniversalArticulationState( universal_articulation_ );
	auto values = state.GetJointValues();
	EXPECT_EQ( values.size(), 1 )
	    << "Universal articulation should have 1 joint";
	EXPECT_DOUBLE_EQ( values[0], 0.0 )
	    << "Default joint value should be 0";
}

// ============================================================
// SetState
// ============================================================

TEST_F( UniversalArticulationStateTest, SetState_WithRotation_UpdatesPoseAndValueCorrectly )
{
	Quaternion rotation = Quaternion::FromTwoVectors( Vec3d( 0, 0, 1 ), Vec3d( 0, 1, 0 ) );
    Vec3d expected_axis   = Vec3d( 0, 1, 0 );
    Vec3d expected_origin = Vec3d( 0, 0, 0 );
    VecXd expected_values( 1 );
    expected_values[0] = 0;

	auto state = Model::UniversalArticulationState( universal_articulation_ );
	state.SetState( rotation, expected_origin, expected_values );

	EXPECT_EQ( expected_values, state.GetJointValues() );
	EXPECT_EQ( expected_values[0], state.Value() );
    EXPECT_EQ( expected_values[0], state.GetJointStates()[0]->Value() );

	EXPECT_TRUE( state.Origin().isApprox( expected_origin ) )
	    << "Expected = " << expected_origin.transpose() << std::endl
	    << "Result   = " << state.Origin().transpose() << std::endl;
    EXPECT_TRUE( state.GetJointStates()[0]->Origin().isApprox( expected_origin ) );
	
    
    EXPECT_TRUE( state.GetJointStates()[0]->Axis().isApprox( expected_origin ) );
    EXPECT_TRUE( state.Axis().isApprox( expected_axis ) )
	    << "Expected = " << expected_axis.transpose() << std::endl
	    << "Result   = " << state.Axis().transpose() << std::endl;
}

// ------------------------------------------------------------

TEST_F( UniversalArticulationStateTest, SetState_WithTranslation_UpdatesPoseAndValueCorrectly )
{
	Quaternion rotation = Quaternion::Identity();
    Vec3d expected_axis   = Vec3d( 0, 0, 1 );
    Vec3d expected_origin = Vec3d( 1, 1, 1 );
    VecXd expected_values( 1 );
    expected_values[0] = 0;

	auto state = Model::UniversalArticulationState( universal_articulation_ );
	state.SetState( rotation, expected_origin, expected_values );

    EXPECT_EQ( expected_values, state.GetJointValues() );
	EXPECT_EQ( expected_values[0], state.Value() );
    EXPECT_EQ( expected_values[0], state.GetJointStates()[0]->Value() );

	EXPECT_TRUE( state.Origin().isApprox( expected_origin ) )
	    << "Expected = " << expected_origin.transpose() << std::endl
	    << "Result   = " << state.Origin().transpose() << std::endl;
    EXPECT_TRUE( state.GetJointStates()[0]->Origin().isApprox( expected_origin ) );
	
    
    EXPECT_TRUE( state.GetJointStates()[0]->Axis().isApprox( expected_origin ) );
    EXPECT_TRUE( state.Axis().isApprox( expected_axis ) )
	    << "Expected = " << expected_axis.transpose() << std::endl
	    << "Result   = " << state.Axis().transpose() << std::endl;
}

// ------------------------------------------------------------

TEST_F( UniversalArticulationStateTest, SetState_WithValue_UpdatesPoseAndValueCorrectly )
{
	Quaternion rotation = Quaternion::Identity();
    Vec3d expected_axis   = Vec3d( 0, 0, 1 );
    Vec3d expected_origin = Vec3d( 0, 0, 0 );
    VecXd expected_values( 1 );
    expected_values[0] = 1;

	auto state = Model::UniversalArticulationState( universal_articulation_ );
	state.SetState( rotation, expected_origin, expected_values );

	EXPECT_EQ( expected_values, state.GetJointValues() );
	EXPECT_EQ( expected_values[0], state.Value() );
    EXPECT_EQ( expected_values[0], state.GetJointStates()[0]->Value() );

	EXPECT_TRUE( state.Origin().isApprox( expected_origin ) )
	    << "Expected = " << expected_origin.transpose() << std::endl
	    << "Result   = " << state.Origin().transpose() << std::endl;
    EXPECT_TRUE( state.GetJointStates()[0]->Origin().isApprox( expected_origin ) );
	
    
    EXPECT_TRUE( state.GetJointStates()[0]->Axis().isApprox( expected_origin ) );
    EXPECT_TRUE( state.Axis().isApprox( expected_axis ) )
	    << "Expected = " << expected_axis.transpose() << std::endl
	    << "Result   = " << state.Axis().transpose() << std::endl;
}

// ------------------------------------------------------------

TEST_F( UniversalArticulationStateTest, SetState_WithRotationTranslationValue_UpdatesPoseAndValueCorrectly )
{
	Quaternion rotation = Quaternion::FromTwoVectors( Vec3d( 0, 0, 1 ), Vec3d( 0, 1, 0 ) );
    Vec3d expected_axis   = Vec3d( 0, 1, 0 );
    Vec3d expected_origin = Vec3d( 1, 1, 1 );
    VecXd expected_values( 1 );
    expected_values[0] = 1;

	auto state = Model::UniversalArticulationState( universal_articulation_ );
	state.SetState( rotation, expected_origin, expected_values );

	EXPECT_EQ( expected_values, state.GetJointValues() );
	EXPECT_EQ( expected_values[0], state.Value() );
    EXPECT_EQ( expected_values[0], state.GetJointStates()[0]->Value() );

	EXPECT_TRUE( state.Origin().isApprox( expected_origin ) )
	    << "Expected = " << expected_origin.transpose() << std::endl
	    << "Result   = " << state.Origin().transpose() << std::endl;
    EXPECT_TRUE( state.GetJointStates()[0]->Origin().isApprox( expected_origin ) );
	
    
    EXPECT_TRUE( state.GetJointStates()[0]->Axis().isApprox( expected_origin ) );
    EXPECT_TRUE( state.Axis().isApprox( expected_axis ) )
	    << "Expected = " << expected_axis.transpose() << std::endl
	    << "Result   = " << state.Axis().transpose() << std::endl;
}

// ============================================================
// SetPose
// ============================================================

TEST_F( UniversalArticulationStateTest, SetPose_UpdatesOriginCorrectly )
{
	Quaternion rotation = Quaternion::FromTwoVectors( Vec3d( 0, 0, 1 ), Vec3d( 0, 1, 0 ) );
    Vec3d expected_axis   = Vec3d( 0, 1, 0 );
    Vec3d expected_origin = Vec3d( 1, 1, 1 );

	auto state = Model::UniversalArticulationState( universal_articulation_ );
	auto values = state.GetJointValues();

    state.SetCenterPose( rotation, expected_origin );

	EXPECT_EQ( values, state.GetJointValues() );

	EXPECT_TRUE( state.Origin().isApprox( expected_origin ) )
	    << "Expected = " << expected_origin.transpose() << std::endl
	    << "Result   = " << state.Origin().transpose() << std::endl;
    EXPECT_TRUE( state.GetJointStates()[0]->Origin().isApprox( expected_origin ) );
	
    
    EXPECT_TRUE( state.GetJointStates()[0]->Axis().isApprox( expected_origin ) );
    EXPECT_TRUE( state.Axis().isApprox( expected_axis ) )
	    << "Expected = " << expected_axis.transpose() << std::endl
	    << "Result   = " << state.Axis().transpose() << std::endl;
}

// ============================================================
// Value accessor
// ============================================================

TEST_F( UniversalArticulationStateTest, Value_ReturnsDefaultValueAfterConstruction )
{
	auto state = Model::UniversalArticulationState( universal_articulation_ );
	EXPECT_DOUBLE_EQ( state.Value(), 0.0 )
	    << "Value should return 0 for a newly constructed articulation state";
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

// ------------------------------------------------------------

} // namespace SOArm100::Kinematics::Test
