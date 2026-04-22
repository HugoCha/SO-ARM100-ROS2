#include "Model/Skeleton/PrismaticArticulationState.hpp"

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
// Prismatic Articulation State
// ============================================================

class PrismaticArticulationStateTest : public KinematicTestBase
{
protected:
void SetUp() override
{
	prismatic_joint_ = MakePrismaticJoint( Vec3d( 0, 0, 1 ), Vec3d( 0, 0, 0 ), 0.0, 1.0 );
	std::vector< Model::JointConstPtr > prismatic_joints = { prismatic_joint_ };
	prismatic_articulation_ = std::make_shared< const Model::Articulation >(
		Model::ArticulationType::Prismatic,
		prismatic_joints,
		Vec3d( 0, 0, 0 ) );
	bone_off_axis_ = std::make_shared< const Model::Bone >( Vec3d( 0, 0, 0 ), Vec3d( 1, 0, 0 ) );
	bone_on_axis_ = std::make_shared< const Model::Bone >( Vec3d( 0, 0, 0 ), Vec3d( 0, 0, 0.1 ) );
}

void TearDown() override {
}

Model::JointConstPtr prismatic_joint_;
Model::ArticulationConstPtr prismatic_articulation_;
Model::BoneConstPtr bone_on_axis_;
Model::BoneConstPtr bone_off_axis_;
};

// ============================================================
// Constructor
// ============================================================

TEST_F( PrismaticArticulationStateTest, Constructor_InitialOriginMatchesArticulationCenter )
{
	auto state = Model::PrismaticArticulationState( prismatic_articulation_ );
	EXPECT_TRUE( state.Origin().isApprox( prismatic_articulation_->Center() ) )
	    << "Initial origin should match the articulation's center";
}

// ------------------------------------------------------------

TEST_F( PrismaticArticulationStateTest, Constructor_InitialAxisMatchesLastJointAxis )
{
	// For revolute, the axis should be the revolute joint's axis
	auto state = Model::PrismaticArticulationState( prismatic_articulation_ );
	EXPECT_TRUE( state.Axis().isApprox( prismatic_articulation_->Axis() ) )
	    << "Axis should match the last joint's axis";
}

// ------------------------------------------------------------

TEST_F( PrismaticArticulationStateTest, Constructor_GetArticulationReturnsCorrectArticulation )
{
	auto state = Model::PrismaticArticulationState( prismatic_articulation_ );
	EXPECT_EQ( state.GetArticulation(), prismatic_articulation_ )
	    << "GetArticulation should return the same articulation passed to constructor";
}

// ============================================================
// GetJointValues
// ============================================================

TEST_F( PrismaticArticulationStateTest, GetJointValues_ReturnsVectorOfDefaultValues )
{
	auto state = Model::PrismaticArticulationState( prismatic_articulation_ );
	auto values = state.GetJointValues();
	EXPECT_EQ( values.size(), 1 )
	    << "Revolute articulation should have 1 joint";
	EXPECT_DOUBLE_EQ( values[0], 0.0 )
	    << "Default joint value should be 0";
}

// ============================================================
// SetState
// ============================================================

TEST_F( PrismaticArticulationStateTest, SetState_WithRotation_UpdatesPoseAndValueCorrectly )
{
	Quaternion rotation = Quaternion::FromTwoVectors( Vec3d( 0, 0, 1 ), Vec3d( 0, 1, 0 ) );
    Vec3d expected_axis   = Vec3d( 0, 1, 0 );
    Vec3d expected_origin = Vec3d( 0, 0, 0 );
    VecXd expected_values( 1 );
    expected_values[0] = 0;

	auto state = Model::PrismaticArticulationState( prismatic_articulation_ );
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

TEST_F( PrismaticArticulationStateTest, SetState_WithTranslation_UpdatesPoseAndValueCorrectly )
{
	Quaternion rotation = Quaternion::Identity();
    Vec3d expected_axis   = Vec3d( 0, 0, 1 );
    Vec3d expected_origin = Vec3d( 1, 1, 1 );
    VecXd expected_values( 1 );
    expected_values[0] = 0;

	auto state = Model::PrismaticArticulationState( prismatic_articulation_ );
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

TEST_F( PrismaticArticulationStateTest, SetState_WithValue_UpdatesPoseAndValueCorrectly )
{
	Quaternion rotation = Quaternion::Identity();
    Vec3d expected_axis   = Vec3d( 0, 0, 1 );
    Vec3d expected_origin = Vec3d( 0, 0, 0 );
    VecXd expected_values( 1 );
    expected_values[0] = 1;

	auto state = Model::PrismaticArticulationState( prismatic_articulation_ );
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

TEST_F( PrismaticArticulationStateTest, SetState_WithRotationTranslationValue_UpdatesPoseAndValueCorrectly )
{
	Quaternion rotation = Quaternion::FromTwoVectors( Vec3d( 0, 0, 1 ), Vec3d( 0, 1, 0 ) );
    Vec3d expected_axis   = Vec3d( 0, 1, 0 );
    Vec3d expected_origin = Vec3d( 1, 1, 1 );
    VecXd expected_values( 1 );
    expected_values[0] = 1;

	auto state = Model::PrismaticArticulationState( prismatic_articulation_ );
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

TEST_F( PrismaticArticulationStateTest, SetPose_UpdatesOriginCorrectly )
{
	Quaternion rotation = Quaternion::FromTwoVectors( Vec3d( 0, 0, 1 ), Vec3d( 0, 1, 0 ) );
    Vec3d expected_axis   = Vec3d( 0, 1, 0 );
    Vec3d expected_origin = Vec3d( 1, 1, 1 );

	auto state = Model::PrismaticArticulationState( prismatic_articulation_ );
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

TEST_F( PrismaticArticulationStateTest, Value_ReturnsDefaultValueAfterConstruction )
{
	auto state = Model::PrismaticArticulationState( prismatic_articulation_ );
	EXPECT_DOUBLE_EQ( state.Value(), 0.0 )
	    << "Value should return 0 for a newly constructed articulation state";
}

// ============================================================
// Apply Constraints
// ============================================================

TEST_F( PrismaticArticulationStateTest, ApplyConstraints_BoneOnAxisWithinLimits_ExpectedResult )
{
	auto state = Model::PrismaticArticulationState( prismatic_articulation_ );

	auto bone_state = Model::BoneState( bone_on_axis_ );
	bone_state.Origin() = Vec3d::Zero();
	bone_state.Direction() = Vec3d( 0.5, 0.5, 0.5 );

	auto rotation = Quaternion::Identity();
	auto center = Vec3d::Zero();
	state.ApplyConstraints( bone_state );

	Vec3d expected_bone_origin = Vec3d::Zero();
	Vec3d expected_bone_direction = Vec3d( 0, 0, 0.5 );

	EXPECT_TRUE( bone_state.Origin().isApprox( expected_bone_origin ) )
	    << "Expected Origin   = " << expected_bone_origin.transpose() << std::endl
	    << "Constraint Origin = " << bone_state.Origin().transpose() << std::endl;

	EXPECT_TRUE( bone_state.Direction().isApprox( expected_bone_direction ) )
	    << "Expected Direction   = " << expected_bone_direction.transpose() << std::endl
	    << "Constraint Direction = " << bone_state.Direction().transpose() << std::endl;
}

// ------------------------------------------------------------

TEST_F( PrismaticArticulationStateTest, ApplyConstraints_BoneOnAxisAboveUpperLimit_ExpectedResult )
{
	auto state = Model::PrismaticArticulationState( prismatic_articulation_ );

	auto bone_state = Model::BoneState( bone_on_axis_ );
	bone_state.Origin() = Vec3d::Zero();
	bone_state.Direction() = Vec3d( 0.5, 0.5, 1.5 );

	auto rotation = Quaternion::Identity();
	auto center = Vec3d::Zero();
	state.ApplyConstraints( bone_state );

	Vec3d expected_bone_origin = Vec3d::Zero();
	Vec3d expected_bone_direction = Vec3d( 0, 0, 1.1 );

	EXPECT_TRUE( bone_state.Origin().isApprox( expected_bone_origin ) )
	    << "Expected Origin   = " << expected_bone_origin.transpose() << std::endl
	    << "Constraint Origin = " << bone_state.Origin().transpose() << std::endl;

	EXPECT_TRUE( bone_state.Direction().isApprox( expected_bone_direction ) )
	    << "Expected Direction   = " << expected_bone_direction.transpose() << std::endl
	    << "Constraint Direction = " << bone_state.Direction().transpose() << std::endl;
}

// ------------------------------------------------------------

TEST_F( PrismaticArticulationStateTest, ApplyConstraints_BoneOnAxisBelowLowerLimit_ExpectedResult )
{
	auto state = Model::PrismaticArticulationState( prismatic_articulation_ );

	auto bone_state = Model::BoneState( bone_on_axis_ );
	bone_state.Origin() = Vec3d::Zero();
	bone_state.Direction() = Vec3d( 0.5, 0.5, -1.5 );

	auto rotation = Quaternion::Identity();
	auto center = Vec3d::Zero();
	state.ApplyConstraints( bone_state );

	Vec3d expected_bone_origin = Vec3d::Zero();
	Vec3d expected_bone_direction = Vec3d( 0, 0, 0.1 );

	EXPECT_TRUE( bone_state.Origin().isApprox( expected_bone_origin ) )
	    << "Expected Origin   = " << expected_bone_origin.transpose() << std::endl
	    << "Constraint Origin = " << bone_state.Origin().transpose() << std::endl;

	EXPECT_TRUE( bone_state.Direction().isApprox( expected_bone_direction ) )
	    << "Expected Direction   = " << expected_bone_direction.transpose() << std::endl
	    << "Constraint Direction = " << bone_state.Direction().transpose() << std::endl;
}

// ------------------------------------------------------------

TEST_F( PrismaticArticulationStateTest, ApplyConstraints_BoneOnAxisWithRotationTranslation_ExpectedResult )
{
	auto state = Model::PrismaticArticulationState( prismatic_articulation_ );

	auto bone_state = Model::BoneState( bone_on_axis_ );
	bone_state.Origin() = Vec3d( 1.1, 1, 1 );
	bone_state.Direction() = Vec3d( 0.5, 0.5, 0.5 );

	auto rotation = Quaternion::FromTwoVectors( Vec3d::UnitZ(), Vec3d::UnitY() );
	auto center = Vec3d::Ones();
	state.ApplyConstraints( bone_state );

	Vec3d expected_bone_origin = Vec3d( 1, 1, 1 );
	Vec3d expected_bone_direction = Vec3d( 0, 0.5, 0 );

	EXPECT_TRUE( bone_state.Origin().isApprox( expected_bone_origin ) )
	    << "Expected Origin   = " << expected_bone_origin.transpose() << std::endl
	    << "Constraint Origin = " << bone_state.Origin().transpose() << std::endl;

	EXPECT_TRUE( bone_state.Direction().isApprox( expected_bone_direction ) )
	    << "Expected Direction   = " << expected_bone_direction.transpose() << std::endl
	    << "Constraint Direction = " << bone_state.Direction().transpose() << std::endl;
}

// ------------------------------------------------------------

TEST_F( PrismaticArticulationStateTest, ApplyConstraints_BoneOffAxisWithinLimits_ExpectedResult )
{
	auto state = Model::PrismaticArticulationState( prismatic_articulation_ );

	auto bone_state = Model::BoneState( bone_off_axis_ );
	bone_state.Origin() = Vec3d::Zero();
	bone_state.Direction() = Vec3d( 0.5, 0.5, 0.5 );

	auto rotation = Quaternion::Identity();
	auto center = Vec3d::Zero();
	state.ApplyConstraints( bone_state );

	Vec3d expected_bone_origin = Vec3d::Zero();
	Vec3d expected_bone_direction = Vec3d( 1, 0, 0.5 );

	EXPECT_TRUE( bone_state.Origin().isApprox( expected_bone_origin ) )
	    << "Expected Origin   = " << expected_bone_origin.transpose() << std::endl
	    << "Constraint Origin = " << bone_state.Origin().transpose() << std::endl;

	EXPECT_TRUE( bone_state.Direction().isApprox( expected_bone_direction ) )
	    << "Expected Direction   = " << expected_bone_direction.transpose() << std::endl
	    << "Constraint Direction = " << bone_state.Direction().transpose() << std::endl;
}

// ------------------------------------------------------------

TEST_F( PrismaticArticulationStateTest, ApplyConstraints_BoneOffAxisAboveUpperLimit_ExpectedResult )
{
	auto state = Model::PrismaticArticulationState( prismatic_articulation_ );

	auto bone_state = Model::BoneState( bone_off_axis_ );
	bone_state.Origin() = Vec3d::Zero();
	bone_state.Direction() = Vec3d( 0.5, 0.5, 1.5 );

	auto rotation = Quaternion::Identity();
	auto center = Vec3d::Zero();
	state.ApplyConstraints( bone_state );

	Vec3d expected_bone_origin = Vec3d::Zero();
	Vec3d expected_bone_direction = Vec3d( 1, 0, 1 );

	EXPECT_TRUE( bone_state.Origin().isApprox( expected_bone_origin ) )
	    << "Expected Origin   = " << expected_bone_origin.transpose() << std::endl
	    << "Constraint Origin = " << bone_state.Origin().transpose() << std::endl;

	EXPECT_TRUE( bone_state.Direction().isApprox( expected_bone_direction ) )
	    << "Expected Direction   = " << expected_bone_direction.transpose() << std::endl
	    << "Constraint Direction = " << bone_state.Direction().transpose() << std::endl;
}

// ------------------------------------------------------------

TEST_F( PrismaticArticulationStateTest, ApplyConstraints_BoneOffAxisBelowLowerLimit_ExpectedResult )
{
	auto state = Model::PrismaticArticulationState( prismatic_articulation_ );

	auto bone_state = Model::BoneState( bone_off_axis_ );
	bone_state.Origin() = Vec3d::Zero();
	bone_state.Direction() = Vec3d( 0.5, 0.5, -1.5 );

	auto rotation = Quaternion::Identity();
	auto center = Vec3d::Zero();
	state.ApplyConstraints( bone_state );

	Vec3d expected_bone_origin = Vec3d::Zero();
	Vec3d expected_bone_direction = Vec3d( 1, 0, 0 );

	EXPECT_TRUE( bone_state.Origin().isApprox( expected_bone_origin ) )
	    << "Expected Origin   = " << expected_bone_origin.transpose() << std::endl
	    << "Constraint Origin = " << bone_state.Origin().transpose() << std::endl;

	EXPECT_TRUE( bone_state.Direction().isApprox( expected_bone_direction ) )
	    << "Expected Direction   = " << expected_bone_direction.transpose() << std::endl
	    << "Constraint Direction = " << bone_state.Direction().transpose() << std::endl;
}

// ------------------------------------------------------------

TEST_F( PrismaticArticulationStateTest, ApplyConstraints_BoneOffAxisWithRotationTranslation_ExpectedResult )
{
	auto state = Model::PrismaticArticulationState( prismatic_articulation_ );

	auto bone_state = Model::BoneState( bone_off_axis_ );
	bone_state.Origin() = Vec3d( 1.1, 1, 1 );
	bone_state.Direction() = Vec3d( 0.5, 0.5, 0.5 );

	auto rotation = Quaternion::FromTwoVectors( Vec3d::UnitZ(), Vec3d::UnitY() );
	auto center = Vec3d::Ones();
	state.ApplyConstraints( bone_state );

	Vec3d expected_bone_origin = Vec3d( 1, 1, 1 );
	Vec3d expected_bone_direction = Vec3d( 1, 0.5, 0 );

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
