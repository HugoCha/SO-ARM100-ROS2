#include "FABRIK/FabrikAnalyzer.hpp"

#include "Global.hpp"

#include "KinematicTestBase.hpp"
#include "Model/Joint/JointChain.hpp"
#include "Model/Limits.hpp"
#include "Model/Link.hpp"
#include "Model/Twist.hpp"
#include "RobotModelTestData.hpp"
#include "Utils/Converter.hpp"

#include <gtest/gtest.h>
#include <ostream>

namespace SOArm100::Kinematics::Test
{

// ------------------------------------------------------------
// ------------------------------------------------------------

class FabrikAnalyzerTest : public KinematicTestBase
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

TEST_F( FabrikAnalyzerTest, Analyze_Planar2RReturnExpected )
{
	auto robot = Data::GetPlanar2RRobot();
	auto fabrik_group = Model::FabrikAnalyzer::Analyze( *robot->GetChain(), robot->GetHomeConfiguration() );

	std::set< int > expected_fabrik_indices = { 0, 1 };
	Mat4d expected_fabrik_home = robot->GetHomeConfiguration();

	EXPECT_TRUE( fabrik_group.has_value() );
	EXPECT_EQ( expected_fabrik_indices, fabrik_group->indices );
	EXPECT_TRUE( expected_fabrik_home.isApprox( fabrik_group->tip_home ) )

	    << "Expected Home = " << std::endl << expected_fabrik_home << std::endl
	    << "Analyzer Home = " << std::endl << fabrik_group->tip_home << std::endl;
}

// ------------------------------------------------------------

TEST_F( FabrikAnalyzerTest, Analyze_Planar3RReturnExpected )
{
	auto robot = Data::GetPlanar3RRobot();
	auto fabrik_group = Model::FabrikAnalyzer::Analyze( *robot->GetChain(), robot->GetHomeConfiguration() );

	std::set< int > expected_fabrik_indices = { 0, 1 };
	Mat4d expected_fabrik_home = robot->GetChain()->GetActiveJoint( 2 )->OriginTransform();

	EXPECT_TRUE( fabrik_group.has_value() );
	EXPECT_EQ( expected_fabrik_indices, fabrik_group->indices );
	EXPECT_TRUE( expected_fabrik_home.isApprox( fabrik_group->tip_home ) )
	    << "Expected Home = " << std::endl << expected_fabrik_home << std::endl
	    << "Analyzer Home = " << std::endl << fabrik_group->tip_home << std::endl;
}

// ------------------------------------------------------------

TEST_F( FabrikAnalyzerTest, Analyze_PrismaticBaseReturnExpected )
{
	auto robot = Data::GetPrismaticBaseRobot();
	auto fabrik_group = Model::FabrikAnalyzer::Analyze( *robot->GetChain(), robot->GetHomeConfiguration() );

	std::set< int > expected_fabrik_indices = { 0, 1 };
	Mat4d expected_fabrik_home = robot->GetHomeConfiguration();

	EXPECT_TRUE( fabrik_group.has_value() );
	EXPECT_EQ( expected_fabrik_indices, fabrik_group->indices );
	EXPECT_TRUE( expected_fabrik_home.isApprox( fabrik_group->tip_home ) )
	    << "Expected Home = " << std::endl << expected_fabrik_home << std::endl
	    << "Analyzer Home = " << std::endl << fabrik_group->tip_home << std::endl;
}

// ------------------------------------------------------------

TEST_F( FabrikAnalyzerTest, Analyze_RevoluteBaseReturnExpected )
{
	auto robot = Data::GetRevoluteBaseRobot();
	auto fabrik_group = Model::FabrikAnalyzer::Analyze( *robot->GetChain(), robot->GetHomeConfiguration() );

	std::set< int > expected_fabrik_indices = { 1 };
	Mat4d expected_fabrik_home = robot->GetChain()->GetActiveJoint( 2 )->OriginTransform();

	EXPECT_TRUE( fabrik_group.has_value() );
	EXPECT_EQ( expected_fabrik_indices, fabrik_group->indices );
	EXPECT_TRUE( expected_fabrik_home.isApprox( fabrik_group->tip_home ) )
	    << "Expected Home = " << std::endl << expected_fabrik_home << std::endl
	    << "Analyzer Home = " << std::endl << fabrik_group->tip_home << std::endl;
}

// ------------------------------------------------------------

TEST_F( FabrikAnalyzerTest, Analyze_WristReturnExpected )
{
	auto robot = Data::GetWrist1RRobot();
	auto fabrik_group = Model::FabrikAnalyzer::Analyze( *robot->GetChain(), robot->GetHomeConfiguration() );
	EXPECT_FALSE( fabrik_group.has_value() );

	robot = Data::GetWrist2RRobot();
	fabrik_group = Model::FabrikAnalyzer::Analyze( *robot->GetChain(), robot->GetHomeConfiguration() );
	EXPECT_FALSE( fabrik_group.has_value() );

	robot = Data::GetSphericalWristRobot();
	fabrik_group = Model::FabrikAnalyzer::Analyze( *robot->GetChain(), robot->GetHomeConfiguration() );
	std::set< int > expected_fabrik_indices = { 2 };
	Mat4d expected_fabrik_home = robot->GetHomeConfiguration();

	EXPECT_TRUE( fabrik_group.has_value() );
	EXPECT_EQ( expected_fabrik_indices, fabrik_group->indices );
	EXPECT_TRUE( expected_fabrik_home.isApprox( fabrik_group->tip_home ) )
	    << "Expected Home = " << std::endl << expected_fabrik_home << std::endl
	    << "Analyzer Home = " << std::endl << fabrik_group->tip_home << std::endl;
}

// ------------------------------------------------------------

TEST_F( FabrikAnalyzerTest, Analyze_5DofsReturnExpected )
{
	auto robot = Data::GetRevolute_Planar2R_Wrist2R_5DOFsRobot();
	auto fabrik_group = Model::FabrikAnalyzer::Analyze( *robot->GetChain(), robot->GetHomeConfiguration() );
	std::set< int > expected_fabrik_indices = { 1, 2 };
	Mat4d expected_fabrik_home = robot->GetChain()->GetActiveJoint( 4 )->OriginTransform();

	EXPECT_TRUE( fabrik_group.has_value() );
	EXPECT_EQ( expected_fabrik_indices, fabrik_group->indices );
	EXPECT_TRUE( expected_fabrik_home.isApprox( fabrik_group->tip_home ) )
	    << "Expected Home = " << std::endl << expected_fabrik_home << std::endl
	    << "Analyzer Home = " << std::endl << fabrik_group->tip_home << std::endl;
}

// ------------------------------------------------------------

TEST_F( FabrikAnalyzerTest, Analyze_6DofWithSphericalWristReturnExpected )
{
	auto robot = Data::GetRevolute_Planar2R_SphericalWrist_6DOFsRobot();
	auto fabrik_group = Model::FabrikAnalyzer::Analyze( *robot->GetChain(), robot->GetHomeConfiguration() );
	std::set< int > expected_fabrik_indices = { 1, 2 };
	Mat4d expected_fabrik_home = robot->GetChain()->GetActiveJoint( 5 )->OriginTransform();

	EXPECT_TRUE( fabrik_group.has_value() );
	EXPECT_EQ( expected_fabrik_indices, fabrik_group->indices );
	EXPECT_TRUE( expected_fabrik_home.isApprox( fabrik_group->tip_home ) )
	    << "Expected Home = " << std::endl << expected_fabrik_home << std::endl
	    << "Analyzer Home = " << std::endl << fabrik_group->tip_home << std::endl;
}

// ------------------------------------------------------------

TEST_F( FabrikAnalyzerTest, Analyze_6DofWithNonSphericalWristReturnExpected )
{
	auto robot = Data::GetURLikeRobot();
	auto fabrik_group = Model::FabrikAnalyzer::Analyze( *robot->GetChain(), robot->GetHomeConfiguration() );
	std::set< int > expected_fabrik_indices = { 0, 1, 2 };
	Mat4d expected_fabrik_home = robot->GetChain()->GetActiveJoint( 5 )->OriginTransform();

	EXPECT_TRUE( fabrik_group.has_value() );
	EXPECT_EQ( expected_fabrik_indices, fabrik_group->indices );
	EXPECT_TRUE( expected_fabrik_home.isApprox( fabrik_group->tip_home ) )
	    << "Expected Home = " << std::endl << expected_fabrik_home << std::endl
	    << "Analyzer Home = " << std::endl << fabrik_group->tip_home << std::endl;
}

// ------------------------------------------------------------

TEST_F( FabrikAnalyzerTest, Analyze_ZYZRobotReturnExpected )
{
	auto robot = Data::GetZYZRevoluteRobot();
	auto fabrik_group = Model::FabrikAnalyzer::Analyze( *robot->GetChain(), robot->GetHomeConfiguration() );
	std::set< int > expected_fabrik_indices = { 0, 1 };
	Mat4d expected_fabrik_home = robot->GetHomeConfiguration();

	EXPECT_TRUE( fabrik_group.has_value() );
	EXPECT_EQ( expected_fabrik_indices, fabrik_group->indices );
	EXPECT_TRUE( expected_fabrik_home.isApprox( fabrik_group->tip_home ) )
	    << "Expected Home = " << std::endl << expected_fabrik_home << std::endl
	    << "Analyzer Home = " << std::endl << fabrik_group->tip_home << std::endl;
}

// ------------------------------------------------------------

TEST_F( FabrikAnalyzerTest, Analyze_3ZRobot_HomeOnJointAxis_ReturnExpected )
{
	auto chain = Model::JointChain( 3 );

	chain.Add(
		Model::Twist( Vec3d::UnitZ(), Vec3d::Zero() ),
		Model::Link( Mat4d::Identity(), 0.5 ),
		Model::Limits()
		);

	chain.Add(
		Model::Twist( Vec3d::UnitZ(), Vec3d( 0, 0, 0.5 ) ),
		Model::Link( ToTransformMatrix( Vec3d( 0, 0, 0.5 ) ), 0.5 ),
		Model::Limits()
		);

	chain.Add(
		Model::Twist( Vec3d::UnitZ(), Vec3d( 0, 0, 1 ) ),
		Model::Link( ToTransformMatrix( Vec3d( 0, 0, 1 ) ), 0 ),
		Model::Limits()
		);

	Mat4d home = ToTransformMatrix( Vec3d( 0, 0, 1.5 ) );

	auto fabrik_group = Model::FabrikAnalyzer::Analyze( chain, home );

	EXPECT_FALSE( fabrik_group.has_value() );
}

// ------------------------------------------------------------

TEST_F( FabrikAnalyzerTest, Analyze_7DofsRobot_ReturnExpected )
{
	auto chain = std::make_unique< Model::JointChain >( 7 );

	// Base Joint
	Vec3d origin      = Vec3d( 0, 0, 0.0 );
	Vec3d next_origin = Vec3d( 0, 0.2, 0.2 );
	Vec3d axis = Vec3d::UnitZ();
	chain->Add(
		Model::Twist( axis, origin ),
		Model::Link( ToTransformMatrix( origin ), ToTransformMatrix( next_origin ) ),
		Model::Limits( -M_PI, M_PI )
		);

	// Planar Joints 3 Y-axis revolute joints
	origin = next_origin;
	next_origin = Vec3d( 0, 0.2, 0.6 );
	axis = Vec3d::UnitY();
	chain->Add(
		Model::Twist( axis, origin ),
		Model::Link( ToTransformMatrix( origin ), ToTransformMatrix( next_origin ) ),
		Model::Limits( -M_PI / 2, M_PI / 2 )
		);

	origin = next_origin;
	next_origin = Vec3d( 0.4, 0, 0.6 );
	axis = Vec3d::UnitY();
	chain->Add(
		Model::Twist( axis, origin ),
		Model::Link( ToTransformMatrix( origin ), ToTransformMatrix( next_origin ) ),
		Model::Limits( -M_PI / 2, M_PI / 2 )
		);

	// Wrist 2 revolute joints
	origin = next_origin;
	next_origin = Vec3d( 0.4, 0.2, 0.6 );
	axis = Vec3d::UnitY();
	chain->Add(
		Model::Twist( axis, origin ),
		Model::Link( ToTransformMatrix( origin ), ToTransformMatrix( next_origin ) ),
		Model::Limits( -M_PI, M_PI )
		);

	origin = next_origin;
	next_origin = Vec3d( 0.5, 0.2, 0.6 );
	axis = Vec3d::UnitX();
	chain->Add(
		Model::Twist( axis, origin ),
		Model::Link( ToTransformMatrix( origin ), ToTransformMatrix( next_origin ) ),
		Model::Limits( -M_PI, M_PI )
		);

	origin = next_origin;
	next_origin = Vec3d( 0.5, 0.2, 0.5 );
	axis = Vec3d::UnitZ();
	chain->Add(
		Model::Twist( axis, origin ),
		Model::Link( ToTransformMatrix( origin ), ToTransformMatrix( next_origin ) ),
		Model::Limits( -M_PI, M_PI )
		);


	origin = next_origin;
	axis = Vec3d::UnitX();
	chain->Add(
		Model::Twist( axis, origin ),
		Model::Link( ToTransformMatrix( origin ), ToTransformMatrix( next_origin ) ),
		Model::Limits( -M_PI, M_PI )
		);

	Mat4d home = ToTransformMatrix( Vec3d( 0.5, 0.2, 0.4 ) );

	auto fabrik_group = Model::FabrikAnalyzer::Analyze( *chain, home );

	std::set< int > expected_fabrik_indices = { 0, 1, 4, 6 };
	Mat4d expected_fabrik_home = home;

	EXPECT_TRUE( fabrik_group.has_value() );
	EXPECT_EQ( expected_fabrik_indices, fabrik_group->indices );
	EXPECT_TRUE( expected_fabrik_home.isApprox( fabrik_group->tip_home ) )
	    << "Expected Home = " << std::endl << expected_fabrik_home << std::endl
	    << "Analyzer Home = " << std::endl << fabrik_group->tip_home << std::endl;
}

// ------------------------------------------------------------

TEST_F( FabrikAnalyzerTest, Analyze_3ZRobot_HomeDifferentLastJointAxis_ReturnExpected )
{
	auto chain = Model::JointChain( 3 );

	chain.Add(
		Model::Twist( Vec3d::UnitZ(), Vec3d::Zero() ),
		Model::Link( Mat4d::Identity(), 0.5 ),
		Model::Limits()
		);

	chain.Add(
		Model::Twist( Vec3d::UnitZ(), Vec3d( 0, 0, 0.5 ) ),
		Model::Link( ToTransformMatrix( Vec3d( 0, 0, 0.5 ) ), 0.5 ),
		Model::Limits()
		);

	chain.Add(
		Model::Twist( Vec3d::UnitZ(), Vec3d( 0, 0, 1 ) ),
		Model::Link( ToTransformMatrix( Vec3d( 0, 0, 1 ) ), 0 ),
		Model::Limits()
		);

	Mat4d home = ToTransformMatrix( Vec3d( 0.1, 0, 1 ) );

	auto fabrik_group = Model::FabrikAnalyzer::Analyze( chain, home );

	std::set< int > expected_fabrik_indices = { 2 };
	Mat4d expected_fabrik_home = home;

	EXPECT_TRUE( fabrik_group.has_value() );
	EXPECT_EQ( expected_fabrik_indices, fabrik_group->indices );
	EXPECT_TRUE( expected_fabrik_home.isApprox( fabrik_group->tip_home ) )
	    << "Expected Home = " << std::endl << expected_fabrik_home << std::endl
	    << "Analyzer Home = " << std::endl << fabrik_group->tip_home << std::endl;
}

// ------------------------------------------------------------

}