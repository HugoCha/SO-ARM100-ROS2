#include "ModelAnalyzer/WristAnalyzer.hpp"

#include "Global.hpp"
#include "KinematicTestBase.hpp"
#include "RobotModelTestData.hpp"

#include "Model/Joint/JointGroup.hpp"
#include "Utils/Converter.hpp"
#include "Utils/KinematicsUtils.hpp"

#include <gtest/gtest.h>
#include <ostream>

namespace SOArm100::Kinematics::Test
{

// ------------------------------------------------------------
// ------------------------------------------------------------

class WristAnalyzerTest : public KinematicTestBase
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

TEST_F( WristAnalyzerTest, Analyze_OneAxisRevoluteWrist )
{
	auto model = Data::GetWrist1RRobot();
	auto chain = model->GetChain();
	auto home_config = model->GetHomeConfiguration();
	auto topology = model->GetTopology().Get( Model::wrist_name );

	auto wrist_group = Model::WristAnalyzer::Analyze( *chain, home_config );

	EXPECT_TRUE( wrist_group.has_value() );
	EXPECT_EQ( 0, wrist_group->FirstIndex() );
	EXPECT_EQ( 0, wrist_group->LastIndex() );
	EXPECT_EQ( 1, wrist_group->Size() );
	EXPECT_TRUE( topology->tip_home.isApprox( wrist_group->tip_home ) )
		<< "Expected \n" << topology->tip_home << std::endl
		<< "Result \n" << wrist_group->tip_home;
}

// ------------------------------------------------------------

TEST_F( WristAnalyzerTest, Analyze_TwoAxisRevoluteWrist )
{
	auto model = Data::GetWrist2RRobot();
	auto chain = model->GetChain();
	auto home_config = model->GetHomeConfiguration();
	auto topology = model->GetTopology().Get( Model::wrist_name );

	auto wrist_group = Model::WristAnalyzer::Analyze( *chain, home_config );

	EXPECT_TRUE( wrist_group.has_value() );
	EXPECT_EQ( 0, wrist_group->FirstIndex() );
	EXPECT_EQ( 1, wrist_group->LastIndex() );
	EXPECT_EQ( 2, wrist_group->Size() );
	EXPECT_TRUE( topology->tip_home.isApprox( wrist_group->tip_home ) )
		<< "Expected \n" << topology->tip_home << std::endl
		<< "Result \n" << wrist_group->tip_home;
}

// ------------------------------------------------------------

TEST_F( WristAnalyzerTest, Analyze_ThreeAxisRevoluteWrist )
{
	auto model = Data::GetSphericalWristRobot();
	auto chain = model->GetChain();
	auto home_config = model->GetHomeConfiguration();
	auto topology = model->GetTopology().Get( Model::wrist_name );

	auto wrist_group = Model::WristAnalyzer::Analyze( *chain, home_config );

	EXPECT_TRUE( wrist_group.has_value() );
	EXPECT_EQ( 0, wrist_group->FirstIndex() );
	EXPECT_EQ( 2, wrist_group->LastIndex() );
	EXPECT_EQ( 3, wrist_group->Size() );
	EXPECT_TRUE( topology->tip_home.isApprox( wrist_group->tip_home ) )
		<< "Expected \n" << topology->tip_home << std::endl
		<< "Result \n" << wrist_group->tip_home;
}

// ------------------------------------------------------------

TEST_F( WristAnalyzerTest, Analyze_FourAxisChain_ThreeAxisRevoluteWrist )
{
	Mat4d home_config = ToTransformMatrix( Vec3d( 2, 0, 0 ) );
	Mat4d wrist_center = ToTransformMatrix( Vec3d( 1.0, 0., 0. ) );
	Mat4d expected_tip_home = home_config * Inverse( wrist_center );

	auto four_joint_chain = CreateSimpleJointChain(
		{ RevoluteJointInfo( Vec3d( 1, 0, 0 ), Vec3d::UnitZ() ), 
		 		RevoluteJointInfo( Vec3d( 1, 0, 0 ), Vec3d::UnitY() ) , 
		 		RevoluteJointInfo( Vec3d( 1, 0, 0 ), Vec3d::UnitX() ) , 
		 		RevoluteJointInfo( Vec3d( 1, 0, 0 ), Vec3d::UnitZ() ) }, 
				 home_config );

	auto wrist_group = Model::WristAnalyzer::Analyze( *four_joint_chain, home_config );

	EXPECT_TRUE( wrist_group.has_value() );
	EXPECT_EQ( 1, wrist_group->FirstIndex() );
	EXPECT_EQ( 3, wrist_group->LastIndex() );
	EXPECT_EQ( 3, wrist_group->Size() );
	EXPECT_TRUE( IsApprox( expected_tip_home, wrist_group->tip_home ) );
}

// ------------------------------------------------------------

TEST_F( WristAnalyzerTest, Analyze_ThreeAxisWrist_WithOffset )
{
	Mat4d home_config = ToTransformMatrix( Vec3d( 1.0, 0.1, 0.1 ) );
	Mat4d wrist_center = ToTransformMatrix( Vec3d( 1.0, 0., 0. ) );
	Mat4d expected_tip_home = home_config * Inverse( wrist_center );

	auto three_axis_chain = CreateSimpleJointChain(
		{ RevoluteJointInfo( Vec3d( 1, 0, 0 ), Vec3d::UnitZ() ), 
		 		RevoluteJointInfo( Vec3d( 1, 0, 0 ), Vec3d::UnitY() ) , 
		 		RevoluteJointInfo( Vec3d( 1, 0, 0 ), Vec3d::UnitX() ) }, 
				 home_config );

	auto wrist_group = Model::WristAnalyzer::Analyze( *three_axis_chain, home_config );

	EXPECT_TRUE( wrist_group.has_value() );
	EXPECT_EQ( 0, wrist_group->FirstIndex() );
	EXPECT_EQ( 2, wrist_group->LastIndex() );
	EXPECT_EQ( 3, wrist_group->Size() );
	EXPECT_TRUE( expected_tip_home.isApprox( wrist_group->tip_home ) )
		<< "Expected \n" << expected_tip_home << std::endl
		<< "Result \n" << wrist_group->tip_home;
}

// ------------------------------------------------------------

TEST_F( WristAnalyzerTest, Analyze_ThreeAxisWrist_WithNonOrthogonalAxes )
{
	Mat4d home_config = ToTransformMatrix( Vec3d( 1.0, 0.1, 0.1 ) );
	Mat4d wrist_center = ToTransformMatrix( Vec3d( 1.0, 0., 0. ) );
	Mat4d expected_tip_home = home_config * Inverse( wrist_center );

	auto three_axis_chain = CreateSimpleJointChain(
		{ RevoluteJointInfo( Vec3d( 1, 0, 0 ), Vec3d::UnitZ() ), 
		 		RevoluteJointInfo( Vec3d( 1, 0, 0 ), Vec3d( 1, 0.1, 0.1 ) ) , 
		 		RevoluteJointInfo( Vec3d( 1, 0, 0 ), Vec3d( 0.1, 1, 0.1 ) ) }, 
				 home_config );

	auto wrist_group = Model::WristAnalyzer::Analyze( *three_axis_chain, home_config );

	EXPECT_TRUE( wrist_group.has_value() );
	EXPECT_EQ( 0, wrist_group->FirstIndex() );
	EXPECT_EQ( 2, wrist_group->LastIndex() );
	EXPECT_EQ( 3, wrist_group->Size() );
	EXPECT_TRUE( expected_tip_home.isApprox( wrist_group->tip_home ) )
		<< "Expected \n" << expected_tip_home << std::endl
		<< "Result \n" << wrist_group->tip_home;
}

// ------------------------------------------------------------

}
