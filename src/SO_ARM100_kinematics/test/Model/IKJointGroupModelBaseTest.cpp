#include <gtest/gtest.h>

#include "Global.hpp"
#include "RobotModelTestData.hpp"

#include "Model/IKJointGroupModelBase.hpp"
#include "Model/Joint/JointGroup.hpp"
#include "Model/KinematicModel.hpp"

namespace SOArm100::Kinematics::Test
{

// ------------------------------------------------------------

class IKJointGroupModelBaseTest : public ::testing::Test
{
protected:
void SetUp() override {
	model_ = Data::GetRevolute_Planar2R_SphericalWrist_6DOFsRobot();
}

void TearDown() override {
	// Clean up if needed
}

Model::KinematicModelConstPtr model_;
};

// ------------------------------------------------------------

TEST_F( IKJointGroupModelBaseTest, Constructor )
{
	auto planar_group =
		Model::PlanarNRJointGroup(
			1,
			2,
			Mat4d::Identity() );

	Model::IKJointGroupModelBase heuristic(
		model_,
		planar_group );

	// Check if the group is correctly set
	ASSERT_EQ( heuristic.GetGroup().name, "planar_NR" );
	ASSERT_EQ( heuristic.GetGroup().Size(), 2 );
	ASSERT_EQ( heuristic.GetGroup().FirstIndex(), 1 );
	ASSERT_EQ( heuristic.GetGroup().LastIndex(), 2 );
}

// ------------------------------------------------------------

TEST_F( IKJointGroupModelBaseTest, GetAncestor_Valid )
{
	auto planar_group =
		Model::PlanarNRJointGroup(
			1,
			2,
			Mat4d::Identity() );

	Model::IKJointGroupModelBase heuristic(
		model_,
		planar_group );

	auto ancestor = heuristic.GetAncestor();
	ASSERT_TRUE( ancestor.has_value() );
	ASSERT_EQ( ancestor->name, "ancestor" );
	ASSERT_EQ( ancestor->FirstIndex(), 0 );
	ASSERT_EQ( ancestor->Size(), 1 );
}

// ------------------------------------------------------------

TEST_F( IKJointGroupModelBaseTest, GetSuccessor_Valid )
{
	auto planar_group =
		Model::PlanarNRJointGroup(
			1,
			2,
			Mat4d::Identity() );

	Model::IKJointGroupModelBase heuristic(
		model_,
		planar_group );

	auto successor = heuristic.GetSuccessor();
	ASSERT_TRUE( successor.has_value() );
	ASSERT_EQ( successor->name, "successor" );
	ASSERT_EQ( successor->FirstIndex(), 3 );
	ASSERT_EQ( successor->LastIndex(), 5 );
	ASSERT_EQ( successor->Size(), 3 );
}

// ------------------------------------------------------------

TEST_F( IKJointGroupModelBaseTest, GetAncestor_NoAncestor )
{
	Model::RevoluteBaseJointGroup base( model_->GetHomeConfiguration() );
	Model::IKJointGroupModelBase heuristic(
		model_,
		base );

	auto ancestor = heuristic.GetAncestor();
	ASSERT_FALSE( ancestor.has_value() );
}

// ------------------------------------------------------------

TEST_F( IKJointGroupModelBaseTest, GetSuccessor_NoSuccessor )
{
	Model::WristJointGroup wrist(
		3,
		3,
		model_->GetHomeConfiguration() );

	Model::IKJointGroupModelBase heuristic(
		model_,
		wrist );

	auto successor = heuristic.GetSuccessor();
	ASSERT_FALSE( successor.has_value() );
}

// ------------------------------------------------------------

}
