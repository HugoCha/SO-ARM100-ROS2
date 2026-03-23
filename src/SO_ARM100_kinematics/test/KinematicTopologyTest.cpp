#include <gtest/gtest.h>

#include "Model/KinematicTopology.hpp"
#include "Global.hpp"
#include "Model/JointGroup.hpp"

namespace SOArm100::Kinematics::Test
{

// ------------------------------------------------------------

class KinematicTopologyTest : public ::testing::Test
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

TEST_F( KinematicTopologyTest, Get_ReturnExpectGroup )
{
	Model::KinematicTopology topology;
	Model::JointGroup group = Model::JointGroup::CreateFromRange(
		"name", 0, 3, Mat4d::Identity() );

	topology.Add( group );

	auto group_topology = topology.Get( "name" );
	EXPECT_TRUE( group_topology.has_value() );
	EXPECT_EQ( group_topology->FirstIndex(), 0 );
	EXPECT_EQ( group_topology->LastIndex(), 2 );
	EXPECT_EQ( group_topology->Size(), 3 );
	EXPECT_EQ( group_topology->name, "name" );
}

// ------------------------------------------------------------

TEST_F( KinematicTopologyTest, Get_ReturnNoGroup )
{
	Model::KinematicTopology topology;
	Model::JointGroup group = Model::JointGroup::CreateFromRange(
		"name", 0, 3, Mat4d::Identity() );

	topology.Add( group );

	auto group_topology = topology.Get( "ame" );
	EXPECT_FALSE( group_topology.has_value() );
}

// ------------------------------------------------------------

}