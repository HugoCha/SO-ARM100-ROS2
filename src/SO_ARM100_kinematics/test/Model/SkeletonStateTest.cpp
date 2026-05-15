#include "Model/Skeleton/SkeletonState.hpp"

#include "Global.hpp"
#include "Model/Skeleton/Skeleton.hpp"
#include "RobotModelTestData.hpp"
#include "KinematicTestBase.hpp"
#include "Model/Skeleton/BoneState.hpp"
#include "Utils/KinematicsUtils.hpp"
#include "Utils/StringConverter.hpp"

#include <gtest/gtest.h>
#include <memory>
#include <vector>
#include <cmath>

namespace SOArm100::Kinematics::Test
{

class SkeletonStateTest : public KinematicTestBase
{
protected:
void SetUp() override
{
	// Use the ZYZRevolute_robot_ for most tests
	skeleton_ = Data::GetZYZRevoluteRobot()->GetSkeleton();
	skeleton_state_ = std::make_unique< Model::SkeletonState >( skeleton_ );
}

void TearDown() override {
}

const Model::Skeleton* skeleton_;
std::unique_ptr< Model::SkeletonState > skeleton_state_;
random_numbers::RandomNumberGenerator rng_;
};

// ============================================================
// Constructor
// ============================================================

TEST_F( SkeletonStateTest, Constructor_InitializesArticulationStates )
{
	EXPECT_EQ( skeleton_->ArticulationCount(), skeleton_state_->GetArticulationStates().size() );
	for ( int i = 0; i < skeleton_->ArticulationCount(); i++ )
	{
		EXPECT_EQ( skeleton_->Articulation( i )->GetType(),
		           skeleton_state_->GetArticulationStates()[i]->GetArticulation()->GetType() );
	}
}

// ============================================================
// GetBoneStates
// ============================================================

TEST_F( SkeletonStateTest, GetBoneStates_ReturnsCorrectBoneStatesForHomeConfiguration )
{
	auto bone_states = skeleton_state_->GetBoneStates();
	EXPECT_EQ( skeleton_->BonesCount(), bone_states.size() );

	// At home configuration, all joint values are zero
	// So the global transform of each articulation should be identity (except for translation)
	for ( int i = 0; i < skeleton_->BonesCount(); i++ )
	{
		const auto& bone = skeleton_->Bone( i );
		const auto& articulation = skeleton_->Articulation( i );
		const auto& bone_state = bone_states[i];

		// Check origin and direction
		EXPECT_TRUE( bone_state.Origin().isApprox( articulation->Center() ) );
		EXPECT_TRUE( bone_state.Direction().isApprox( bone->Direction() ) );
	}
}

// ============================================================
// GetJointValues
// ============================================================

TEST_F( SkeletonStateTest, GetJointValues_ReturnsZeroForHomeConfiguration )
{
	auto joint_values = skeleton_state_->GetJointValues();
	EXPECT_EQ( skeleton_->JointCount(), joint_values.size() );
	for ( int i = 0; i < joint_values.size(); i++ )
	{
		EXPECT_DOUBLE_EQ( 0.0, joint_values[i] );
	}
}

// ============================================================
// SetState
// ============================================================

TEST_F( SkeletonStateTest, SetState_WithNonZeroJoints_UpdatesBoneStatesCorrectly )
{
	// Set joint values for ZYZRevolute_robot_
	VecXd joints( 3 );
	joints << M_PI / 4, M_PI / 4, M_PI / 4; // theta1, theta2, theta3

	skeleton_state_->SetState( joints );

	auto joint_values = skeleton_state_->GetJointValues();
	EXPECT_TRUE( joint_values.isApprox( joints ) );

	// Check that bone states are updated
	auto bone_states = skeleton_state_->GetBoneStates();

	EXPECT_EQ( bone_states.size(), skeleton_->BonesCount() );

	auto bone = skeleton_->Bone( 0 );
	auto bone_state = bone_states[0];
	const auto& T01 = Data::GetZYZRevoluteRobotT01( joints[0] );
	const auto& T12 = Data::GetZYZRevoluteRobotT12( joints[1] );
	const auto& T23 = Data::GetZYZRevoluteRobotT23( joints[2] );
	const auto& T02 = T01 * T12;
	const auto& T03 = T02 * T23;

	Vec3d expected_origin = bone->Origin();
	Vec3d expected_dir = Translation( T02 ) - Translation( T01 );

	EXPECT_TRUE( bone_state.Origin().isApprox( expected_origin ) )
	    << "Expected Origin = " << expected_origin.transpose() << std::endl
	    << "Result   Origin = " << bone_state.Origin().transpose() << std::endl;
	EXPECT_TRUE( bone_state.Direction().isApprox( expected_dir ) )
	    << "Expected Direction = " << expected_dir.transpose() << std::endl
	    << "Result   Direction = " << bone_state.Direction().transpose() << std::endl;

	bone = skeleton_->Bone( 1 );
	bone_state = bone_states[1];
	expected_origin = Translation( T02 );
	expected_dir = Translation( T03 ) - Translation( T02 );

	EXPECT_TRUE( bone_state.Origin().isApprox( expected_origin ) )
	    << "Expected Origin = " << expected_origin.transpose() << std::endl
	    << "Result   Origin = " << bone_state.Origin().transpose() << std::endl;
	EXPECT_TRUE( bone_state.Direction().isApprox( expected_dir ) )
	    << "Expected Direction = " << expected_dir.transpose() << std::endl
	    << "Result   Direction = " << bone_state.Direction().transpose() << std::endl;
}

// ============================================================
// UpdateValue
// ============================================================

TEST_F( SkeletonStateTest, UpdateValue_WithValidBoneStateIndex0_UpdatesArticulationState )
{
	VecXd joints( 3 );
	joints << M_PI / 6, M_PI / 6, M_PI / 6;
	skeleton_state_->SetState( joints );

	auto bone_state = Model::BoneState( skeleton_->Bone( 0 ) );
	bone_state.Origin() = Vec3d( 0., 0., 0. );
	bone_state.Direction() = Vec3d( 1, 0, 0 ).normalized() * skeleton_->Bone( 0 )->Length();

	skeleton_state_->UpdateValue( bone_state, 0 );

	Vec3d expected_joints;
	expected_joints << 0, M_PI / 6, M_PI / 6;

	const auto& T01 = Data::GetZYZRevoluteRobotT01( expected_joints[0] );
	const auto& T12 = Data::GetZYZRevoluteRobotT12( expected_joints[1] );
	const auto& T23 = Data::GetZYZRevoluteRobotT23( expected_joints[2] );
	const auto& T02 = T01 * T12;
	const auto& T03 = T02 * T23;

	// Articulation States
	auto articulation_state = skeleton_state_->GetArticulationStates()[0];

	EXPECT_EQ( articulation_state->GetJointValues().size(), 1 );
	EXPECT_EQ( articulation_state->GetJointValues()[0], expected_joints[0] );
	EXPECT_TRUE( articulation_state->GlobalTransform().translation().isApprox( Vec3d::Zero() ) );
	EXPECT_TRUE( articulation_state->LocalTransform().translation().isApprox( Vec3d::Zero() ) );

	articulation_state = skeleton_state_->GetArticulationStates()[1];

	EXPECT_EQ( articulation_state->GetJointValues().size(), 1 );
	EXPECT_EQ( articulation_state->GetJointValues()[0], expected_joints[1] );
	EXPECT_TRUE( articulation_state->GlobalTransform().translation().isApprox( Translation( T02 ) ) )
	    << "Expected Translation = " << Translation( T02 ).transpose() << std::endl
	    << "Result   Translation = " << articulation_state->GlobalTransform().translation().transpose() << std::endl;
	EXPECT_TRUE( articulation_state->LocalTransform().translation().isApprox( Vec3d::Zero() ) );

	articulation_state = skeleton_state_->GetArticulationStates()[2];

	EXPECT_EQ( articulation_state->GetJointValues().size(), 1 );
	EXPECT_EQ( articulation_state->GetJointValues()[0], expected_joints[2] );
	EXPECT_TRUE( articulation_state->GlobalTransform().translation().isApprox( Translation( T03 ) ) )
	    << "Expected Translation = " << Translation( T03 ).transpose() << std::endl
	    << "Result   Translation = " << articulation_state->GlobalTransform().translation().transpose() << std::endl;
	EXPECT_TRUE( articulation_state->LocalTransform().translation().isApprox( Vec3d::Zero() ) );
}

// ------------------------------------------------------------

TEST_F( SkeletonStateTest, UpdateValue_WithValidBoneStateIndex1_UpdatesArticulationState )
{
	VecXd joints( 3 );
	joints << M_PI / 6, M_PI / 6, M_PI / 6;
	skeleton_state_->SetState( joints );

	auto bone_state = Model::BoneState( skeleton_->Bone( 1 ) );
	bone_state.Origin() = skeleton_->Bone( 1 )->Origin();
	bone_state.Direction() = Vec3d( 1, 0, 0 ).normalized() * skeleton_->Bone( 1 )->Length();

	skeleton_state_->UpdateValue( bone_state, 1 );

	Vec3d expected_joints;
	expected_joints << M_PI / 6, 0, M_PI / 6;

	const auto& T01 = Data::GetZYZRevoluteRobotT01( expected_joints[0] );
	const auto& T12 = Data::GetZYZRevoluteRobotT12( expected_joints[1] );
	const auto& T23 = Data::GetZYZRevoluteRobotT23( expected_joints[2] );
	const auto& T02 = T01 * T12;
	const auto& T03 = T02 * T23;

	// Articulation States
	auto articulation_state = skeleton_state_->GetArticulationStates()[0];

	EXPECT_EQ( articulation_state->GetJointValues().size(), 1 );
	EXPECT_EQ( articulation_state->GetJointValues()[0], expected_joints[0] );
	EXPECT_TRUE( articulation_state->GlobalTransform().translation().isApprox( Vec3d::Zero() ) );
	EXPECT_TRUE( articulation_state->LocalTransform().translation().isApprox( Vec3d::Zero() ) );

	articulation_state = skeleton_state_->GetArticulationStates()[1];

	EXPECT_EQ( articulation_state->GetJointValues().size(), 1 );
	EXPECT_EQ( articulation_state->GetJointValues()[0], expected_joints[1] );
	EXPECT_TRUE( articulation_state->GlobalTransform().translation().isApprox( Translation( T02 ) ) )
	    << "Expected Translation = " << Translation( T02 ).transpose() << std::endl
	    << "Result   Translation = " << articulation_state->GlobalTransform().translation().transpose() << std::endl;
	EXPECT_TRUE( articulation_state->LocalTransform().translation().isApprox( Vec3d::Zero() ) );

	articulation_state = skeleton_state_->GetArticulationStates()[2];

	EXPECT_EQ( articulation_state->GetJointValues().size(), 1 );
	EXPECT_EQ( articulation_state->GetJointValues()[0], expected_joints[2] );
	EXPECT_TRUE( articulation_state->GlobalTransform().translation().isApprox( Translation( T03 ) ) )
	    << "Expected Translation = " << Translation( T03 ).transpose() << std::endl
	    << "Result   Translation = " << articulation_state->GlobalTransform().translation().transpose() << std::endl;
	EXPECT_TRUE( articulation_state->LocalTransform().translation().isApprox( Vec3d::Zero() ) );
}

// ------------------------------------------------------------

TEST_F( SkeletonStateTest, UpdateValue_WithValidBoneState_UpdatesBoneState )
{
	VecXd joints( 3 );
	joints << M_PI / 6, M_PI / 6, M_PI / 6;
	skeleton_state_->SetState( joints );

	auto bone_state = Model::BoneState( skeleton_->Bone( 0 ) );
	bone_state.Origin() = Vec3d( 0., 0., 0. );
	bone_state.Direction() = Vec3d( 1, 0, 0 ).normalized() * skeleton_->Bone( 0 )->Length();

	skeleton_state_->UpdateValue( bone_state, 0 );

	Vec3d expected_joints;
	expected_joints << 0, M_PI / 6, M_PI / 6;

	const auto& T01 = Data::GetZYZRevoluteRobotT01( expected_joints[0] );
	const auto& T12 = Data::GetZYZRevoluteRobotT12( expected_joints[1] );
	const auto& T23 = Data::GetZYZRevoluteRobotT23( expected_joints[2] );
	const auto& T02 = T01 * T12;
	const auto& T03 = T02 * T23;

	// Bone States
	auto bone_states = skeleton_state_->GetBoneStates();

	EXPECT_EQ( bone_states.size(), skeleton_->BonesCount() );

	auto bone = skeleton_->Bone( 0 );
	auto result_bone_state = bone_states[0];

	Vec3d expected_origin = bone->Origin();
	Vec3d expected_dir = Translation( T02 ) - Translation( T01 );

	EXPECT_TRUE( result_bone_state.Origin().isApprox( expected_origin ) )
	    << "Expected Origin = " << expected_origin.transpose() << std::endl
	    << "Result   Origin = " << result_bone_state.Origin().transpose() << std::endl;
	EXPECT_TRUE( result_bone_state.Direction().isApprox( expected_dir ) )
	    << "Expected Direction = " << expected_dir.transpose() << std::endl
	    << "Result   Direction = " << result_bone_state.Direction().transpose() << std::endl;

	bone = skeleton_->Bone( 1 );
	result_bone_state = bone_states[1];
	expected_origin = Translation( T02 );
	expected_dir = Translation( T03 ) - Translation( T02 );

	EXPECT_TRUE( result_bone_state.Origin().isApprox( expected_origin ) )
	    << "Expected Origin = " << expected_origin.transpose() << std::endl
	    << "Result   Origin = " << result_bone_state.Origin().transpose() << std::endl;
	EXPECT_TRUE( result_bone_state.Direction().isApprox( expected_dir ) )
	    << "Expected Direction = " << expected_dir.transpose() << std::endl
	    << "Result   Direction = " << result_bone_state.Direction().transpose() << std::endl;
}

// ============================================================
// ApplyConstraint
// ============================================================

TEST_F( SkeletonStateTest, ApplyConstraint_WithValidBoneState_AppliesConstraints )
{
	// Set initial state
	VecXd joints( 3 );
	joints << M_PI / 6, M_PI / 6, M_PI / 6;
	skeleton_state_->SetState( joints );

	// Create a BoneState for the first bone
	auto bone_state = Model::BoneState( skeleton_->Bone( 1 ) );
	bone_state.Origin() = Vec3d( 0.1, 0.1, 0.1 );
	bone_state.Direction() = Vec3d( 1, 0, -1 ).normalized() * skeleton_->Bone( 1 )->Length();

	// Apply constraint
	skeleton_state_->ApplyConstraint( bone_state, 0 );

	Vec3d expected_direction = Vec3d( 0.5, 0, 0 );
	EXPECT_TRUE( bone_state.Direction().isApprox( expected_direction ) )
	    << "Expected Direction = " << expected_direction.transpose() << std::endl
	    << "Result   Direction = " << bone_state.Direction().transpose() << std::endl;
}

// ============================================================
// Edge Cases
// ============================================================

TEST_F( SkeletonStateTest, SetState_WithInvalidJoints_ThrowsException )
{
	VecXd invalid_joints( 2 ); // Wrong size
	EXPECT_THROW( skeleton_state_->SetState( invalid_joints ), std::invalid_argument );
}

// ------------------------------------------------------------

TEST_F( SkeletonStateTest, UpdateValue_WithInvalidIndex_ThrowsException )
{
	auto bone_state = Model::BoneState( skeleton_->Bone( 0 ) );
	EXPECT_THROW( skeleton_state_->UpdateValue( bone_state, -1 ), std::out_of_range );
	EXPECT_THROW( skeleton_state_->UpdateValue( bone_state, skeleton_->ArticulationCount() ), std::out_of_range );
}

// ------------------------------------------------------------

TEST_F( SkeletonStateTest, ApplyConstraint_WithInvalidIndex_ThrowsException )
{
	auto bone_state = Model::BoneState( skeleton_->Bone( 0 ) );
	EXPECT_THROW( skeleton_state_->ApplyConstraint( bone_state, -1 ), std::out_of_range );
	EXPECT_THROW( skeleton_state_->ApplyConstraint( bone_state, skeleton_->ArticulationCount() ), std::out_of_range );
}

// ------------------------------------------------------------

} // namespace SOArm100::Kinematics::Test