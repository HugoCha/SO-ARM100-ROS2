#include "Model/KinematicModel.hpp"
#include "ModelAnalyzer/BaseAnalyzer.hpp"

#include "Global.hpp"

#include "Model/JointChain.hpp"
#include "Model/JointGroup.hpp"
#include "Model/Twist.hpp"
#include "RobotModelTestData.hpp"
#include "Utils/Converter.hpp"

#include <gtest/gtest.h>
#include <optional>

namespace SOArm100::Kinematics::Test
{

// ------------------------------------------------------------
// ------------------------------------------------------------

class BaseAnalyzerTest : public ::testing::Test
{
protected:
void SetUp() override
{
}

void TearDown() override
{
}
};

// ------------------------------------------------------------
// ------------------------------------------------------------

TEST_F( BaseAnalyzerTest, AnalyzeRevoluteBase_ValidInput )
{
	auto model = Data::GetRevoluteBaseRobot();
	Model::WristJointGroup wrist( 
		1, 
		2, 
		ToTransformMatrix( Vec3d( 0, 0, -0.1 ) ) );

	auto base_group = Model::BaseAnalyzer::Analyze( 
		*model->GetChain(), 
		model->GetHomeConfiguration(), 
		wrist );

	Mat4d expected_tip_home = ToTransformMatrix( Vec3d( 1, 0, 1 ) );
	
	EXPECT_TRUE( base_group.has_value() );
	EXPECT_EQ( 0, base_group->FirstIndex() );
	EXPECT_EQ( 0, base_group->LastIndex() );
	EXPECT_EQ( 1, base_group->Size() );
	EXPECT_EQ( Model::revolute_base_name, base_group->name );
	EXPECT_EQ( expected_tip_home, base_group->tip_home );
}

// ------------------------------------------------------------

TEST_F( BaseAnalyzerTest, AnalyzePrismaticBase_ValidInput )
{
	auto model = Data::GetPrismaticBaseRobot();
	Model::WristJointGroup wrist( 
		1, 
		1, 
		ToTransformMatrix( Vec3d( 1, 0, 0 ) ) );

	auto base_group = Model::BaseAnalyzer::Analyze( 
		*model->GetChain(), 
		model->GetHomeConfiguration(), 
		wrist );

	Mat4d expected_tip_home = ToTransformMatrix( Vec3d( 0, 0, 1 ) );
	
	EXPECT_TRUE( base_group.has_value() );
	EXPECT_EQ( 0, base_group->FirstIndex() );
	EXPECT_EQ( 0, base_group->LastIndex() );
	EXPECT_EQ( 1, base_group->Size() );
	EXPECT_EQ( Model::prismatic_base_name, base_group->name );
	EXPECT_EQ( expected_tip_home, base_group->tip_home );
}

// ------------------------------------------------------------

TEST_F( BaseAnalyzerTest, AnalyzeRevoluteBase_NoWristModel )
{
	auto model = Data::GetRevoluteBaseRobot();

	auto base_group = Model::BaseAnalyzer::Analyze( 
		*model->GetChain(), 
		model->GetHomeConfiguration(), 
		std::nullopt );
	
	EXPECT_FALSE( base_group.has_value() );
}

// ------------------------------------------------------------

TEST_F( BaseAnalyzerTest, Analyze_EmptyJointChain )
{
	auto empty_model = Model::KinematicModel::Empty();
	Model::WristJointGroup wrist( 1, 1, Mat4d::Identity() );

	auto base_group = Model::BaseAnalyzer::Analyze( 
		*empty_model.GetChain(), 
		empty_model.GetHomeConfiguration(), 
		std::nullopt );
	
	EXPECT_FALSE( base_group.has_value() );
}

// ------------------------------------------------------------

TEST_F( BaseAnalyzerTest, AnalyzeRevoluteBase_WithDifferentWristPositions )
{
	auto model = Data::GetRevoluteBaseRobot();
	
	Model::WristJointGroup wrist_y( 
		1, 
		1, 
		ToTransformMatrix( Vec3d( 0, 1, 0 ) ) );
	auto home_y = ToTransformMatrix( Vec3d( 0, 1, 1 ));

	auto base_group = Model::BaseAnalyzer::Analyze( 
		*model->GetChain(), 
		home_y, 
		wrist_y );
	Mat4d expected_tip_home = ToTransformMatrix( Vec3d( 0, 0, 1 ) );
	
	ASSERT_TRUE( base_group.has_value() );
	EXPECT_EQ( expected_tip_home, base_group->tip_home );

	// Test with wrist center at (1, 0, 0)
	Model::WristJointGroup wrist_x( 
		1, 
		1, 
		ToTransformMatrix( Vec3d( 1, 0, 0 ) ) );
	auto home_x = ToTransformMatrix( Vec3d( 1, 0, 1 ));

	base_group = Model::BaseAnalyzer::Analyze( 
		*model->GetChain(), 
		home_x, 
		wrist_x );

	ASSERT_TRUE( base_group.has_value() );
	EXPECT_EQ( expected_tip_home, base_group->tip_home );
}

// ------------------------------------------------------------

TEST_F( BaseAnalyzerTest, AnalyzeRevoluteBase_WithDifferentBaseJointAxis )
{
	// Create a joint chain with a different base joint axis
	Model::JointChain joint_chain_y( 2 );
	Vec3d origin = Vec3d::Zero();
	Vec3d axis = Vec3d::UnitY(); // Rotation around Y-axis
	Model::Twist twist_y( axis, origin );
	Model::Link link_y( Mat4d::Identity(), 1 );
	Model::Limits limits_y( -M_PI, M_PI );
	joint_chain_y.Add( twist_y, link_y, limits_y );

	origin = Vec3d::UnitZ();
	axis = Vec3d::UnitZ(); // Rotation around Z-axis
	Model::Twist twist_z( axis, origin );
	Model::Link link_z( ToTransformMatrix( origin ), 1 );
	Model::Limits limits_z( -M_PI, M_PI );
	joint_chain_y.Add( twist_z, link_z, limits_z );

	Mat4d home = ToTransformMatrix( Vec3d( 1, 0, 1 ) );
	
	Model::WristJointGroup wrist( 
		1, 
		1, 
		ToTransformMatrix( Vec3d( 1, 0, 0 ) ) );

	auto base_group = Model::BaseAnalyzer::Analyze( 
		joint_chain_y, 
		home, 
		wrist );

	Mat4d expected_tip_home = ToTransformMatrix( Vec3d( 0, 0, 1 ) );
	
	EXPECT_TRUE( base_group.has_value() );
	EXPECT_EQ( 0, base_group->FirstIndex() );
	EXPECT_EQ( 0, base_group->LastIndex() );
	EXPECT_EQ( 1, base_group->Size() );
	EXPECT_EQ( Model::revolute_base_name, base_group->name );
	EXPECT_EQ( expected_tip_home, base_group->tip_home );
}

// ------------------------------------------------------------

TEST_F( BaseAnalyzerTest, IsConsistent_Valid )
{
	// Revolute
	auto revolute_model = Data::GetRevoluteBaseRobot();
	
	Model::RevoluteBaseJointGroup revolute_base( Mat4d::Identity() );
	ASSERT_TRUE( 
		Model::BaseAnalyzer::CheckConsistency( 
			*revolute_model->GetChain(), 
			revolute_base ) );

	Model::JointGroup revolute_group{ 
		Model::revolute_base_name, 
		{0}, 
		Mat4d::Identity() };
	ASSERT_TRUE( 
		Model::BaseAnalyzer::CheckConsistency( 
			*revolute_model->GetChain(), 
			revolute_group ) );

	// Prismatic
	auto prismatic_model = Data::GetPrismaticBaseRobot();
	
	Model::PrismaticBaseJointGroup prismatic_base( Mat4d::Identity() );
	ASSERT_TRUE( 
		Model::BaseAnalyzer::CheckConsistency( 
			*prismatic_model->GetChain(), 
			prismatic_base ) );

	Model::JointGroup prismatic_group{ 
		Model::prismatic_base_name, 
		{0}, 
		Mat4d::Identity() };
	ASSERT_TRUE( 
		Model::BaseAnalyzer::CheckConsistency(
			*prismatic_model->GetChain(), 
			prismatic_group ) );
}

// ------------------------------------------------------------

TEST_F( BaseAnalyzerTest, IsConsistent_Invalid )
{
	// Revolute
	auto revolute_model = Data::GetRevoluteBaseRobot();
	auto prismatic_model = Data::GetPrismaticBaseRobot();
	
	Model::RevoluteBaseJointGroup revolute_base( Mat4d::Identity() );
	EXPECT_FALSE( 
		Model::BaseAnalyzer::CheckConsistency( 
			*prismatic_model->GetChain(), 
			revolute_base ) );

	// Prismatic
	Model::PrismaticBaseJointGroup prismatic_base( Mat4d::Identity() );
	EXPECT_FALSE( 
		Model::BaseAnalyzer::CheckConsistency( 
			*revolute_model->GetChain(), 
			prismatic_base ) );

	Model::JointGroup invalid = revolute_base;
	invalid.indices = {0, 1 };
	EXPECT_FALSE( 
		Model::BaseAnalyzer::CheckConsistency( 
			*revolute_model->GetChain(), 
			invalid ) );

	invalid = revolute_base;
	invalid.indices = { 1 };
	EXPECT_FALSE( 
		Model::BaseAnalyzer::CheckConsistency( 
			*revolute_model->GetChain(), 
			invalid ) );
	
	invalid = revolute_base;
	invalid.name = "";
	EXPECT_FALSE( 
		Model::BaseAnalyzer::CheckConsistency( 
			*revolute_model->GetChain(), 
			invalid ) );
}

// ------------------------------------------------------------

}