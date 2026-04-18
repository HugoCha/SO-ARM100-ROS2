#include <gtest/gtest.h>

#include "RobotModelTestData.hpp"

#include "Model/Joint/JointGroup.hpp"

namespace SOArm100::Kinematics::Test
{

// ------------------------------------------------------------

class JointGroupTest : public ::testing::Test
{
protected:
void SetUp() override {
	// Set up common test data
}

void TearDown() override {
	// Clean up if needed
}
};

// ------------------------------------------------------------

TEST_F( JointGroupTest, CreateFromRange )
{
	std::string name = "test_group";
	int start = 2;
	int count = 3;
	Mat4d home = Mat4d::Identity();

	Model::JointGroup group = Model::JointGroup::CreateFromRange( name, start, count, home );

	// Check if the group is correctly created
	ASSERT_EQ( group.name, name );
	ASSERT_EQ( group.Size(), count );
	ASSERT_EQ( group.FirstIndex(), start );
	ASSERT_EQ( group.LastIndex(), start + count - 1 );
}

// ------------------------------------------------------------

TEST_F( JointGroupTest, IsConsistent )
{
	auto chain = Data::GetZYZRevoluteRobotJointChain();

	// Valid group
	Model::JointGroup valid_group = Model::JointGroup::CreateFromRange( "valid_group", 1, 2, Mat4d::Identity() );
	ASSERT_TRUE( Model::JointGroup::IsConsistent( chain, valid_group ) );

	// Invalid group (out of bounds)
	Model::JointGroup invalid_group = Model::JointGroup::CreateFromRange( "invalid_group", 8, 3, Mat4d::Identity() );
	ASSERT_FALSE( Model::JointGroup::IsConsistent( chain, invalid_group ) );
}

// ------------------------------------------------------------

TEST_F( JointGroupTest, IsDense )
{
	Model::JointGroup dense_group = Model::JointGroup::CreateFromRange( "dense_group", 2, 3, Mat4d::Identity() );
	ASSERT_TRUE( Model::JointGroup::IsDense( dense_group ) );

	// Create a non-dense group
	std::set< int > non_dense_indices = { 2, 4, 6 };
	Model::JointGroup non_dense_group{ "non_dense_group", non_dense_indices, Mat4d::Identity() };
	ASSERT_FALSE( Model::JointGroup::IsDense( non_dense_group ) );
}

// ------------------------------------------------------------

TEST_F( JointGroupTest, Index )
{
	Model::JointGroup group = Model::JointGroup::CreateFromRange( "test_group", 2, 3, Mat4d::Identity() );

	// Check if the index method returns the correct value
	ASSERT_EQ( group.Index( 0 ), 2 );
	ASSERT_EQ( group.Index( 1 ), 3 );
	ASSERT_EQ( group.Index( 2 ), 4 );
	ASSERT_EQ( group.Index( -1 ), -1 ); // Invalid index
	ASSERT_EQ( group.Index( 3 ), -1 );  // Invalid index
}

// ------------------------------------------------------------

TEST_F( JointGroupTest, GetGroupJoints )
{
	Model::JointGroup group = Model::JointGroup::CreateFromRange( "test_group", 2, 3, Mat4d::Identity() );
	VecXd full_joints = VecXd::LinSpaced( 10, 0.0, 9.0 ); // Example full joints vector

	VecXd group_joints = group.GetGroupJoints( full_joints );

	// Check if the group joints are correctly extracted
	ASSERT_EQ( group_joints.size(), group.Size() );
	ASSERT_EQ( group_joints[0], full_joints[2] );
	ASSERT_EQ( group_joints[1], full_joints[3] );
	ASSERT_EQ( group_joints[2], full_joints[4] );
}

// ------------------------------------------------------------

TEST_F( JointGroupTest, SetGroupJoints )
{
	Model::JointGroup group = Model::JointGroup::CreateFromRange( "test_group", 2, 3, Mat4d::Identity() );
	VecXd full_joints = VecXd::LinSpaced( 10, 0.0, 9.0 ); // Example full joints vector
	VecXd group_joints = VecXd::LinSpaced( 3, 10.0, 12.0 ); // Example group joints vector

	group.SetGroupJoints( group_joints, full_joints );

	// Check if the group joints are correctly set in the full joints vector
	ASSERT_EQ( full_joints[2], group_joints[0] );
	ASSERT_EQ( full_joints[3], group_joints[1] );
	ASSERT_EQ( full_joints[4], group_joints[2] );
}

// ------------------------------------------------------------

TEST_F( JointGroupTest, RevoluteBaseJointGroup )
{
	Mat4d home = Mat4d::Identity();
	Model::RevoluteBaseJointGroup group( home );

	// Check if the group is correctly created
	ASSERT_EQ( group.name, Model::revolute_base_name );
	ASSERT_EQ( group.Size(), 1 );
	ASSERT_EQ( group.FirstIndex(), 0 );
}

// ------------------------------------------------------------

TEST_F( JointGroupTest, PrismaticBaseJointGroup )
{
	Mat4d home = Mat4d::Identity();
	Model::PrismaticBaseJointGroup group( home );

	// Check if the group is correctly created
	ASSERT_EQ( group.name, Model::prismatic_base_name );
	ASSERT_EQ( group.Size(), 1 );
	ASSERT_EQ( group.FirstIndex(), 0 );
}

// ------------------------------------------------------------

TEST_F( JointGroupTest, PlanarNRJointGroup )
{
	Mat4d home = Mat4d::Identity();
	Model::PlanarNRJointGroup group( 0, 3, home );

	// Check if the group is correctly created
	ASSERT_EQ( group.name, Model::planarNR_name );
	ASSERT_EQ( group.Size(), 3 );
	ASSERT_EQ( group.FirstIndex(), 0 );
}

// ------------------------------------------------------------

TEST_F( JointGroupTest, WristJointGroup )
{
	Mat4d home = Mat4d::Identity();
	Model::WristJointGroup group( 0, 3, home );

	// Check if the group is correctly created
	ASSERT_EQ( group.name, Model::wrist_name );
	ASSERT_EQ( group.Size(), 3 );
	ASSERT_EQ( group.FirstIndex(), 0 );
}

// ------------------------------------------------------------

}
