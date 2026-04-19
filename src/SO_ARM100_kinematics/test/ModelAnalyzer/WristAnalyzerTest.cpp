#include "ModelAnalyzer/WristAnalyzer.hpp"

#include "Global.hpp"
#include "KinematicTestBase.hpp"
#include "RobotModelTestData.hpp"

#include "Model/Joint/JointChain.hpp"
#include "Model/Joint/JointGroup.hpp"
#include "Model/Joint/Twist.hpp"
#include "Utils/Converter.hpp"
#include "Utils/KinematicsUtils.hpp"

#include <gtest/gtest.h>

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
	// Create a joint chain with one revolute axis (typical spherical wrist)
	Model::JointChain one_axis_chain( 1 );

	// Joint 1: Rotation around Z-axis
	one_axis_chain.Add(
		Model::Twist( Vec3d( 0, 0, 1 ), Vec3d( 0, 0, 0 ) ),
		Model::Link( Mat4d::Identity(), 0 ),
		Model::Limits( -M_PI, M_PI )
		);

	Mat4d home_config = ToTransformMatrix( Vec3d( 0.5, 0.0, 0.0 ) );

	auto wrist_group = Model::WristAnalyzer::Analyze( one_axis_chain, home_config );

	EXPECT_TRUE( wrist_group.has_value() );
	EXPECT_EQ( 0, wrist_group->FirstIndex() );
	EXPECT_EQ( 0, wrist_group->LastIndex() );
	EXPECT_EQ( 1, wrist_group->Size() );
	EXPECT_EQ( home_config, wrist_group->tip_home );
}

// ------------------------------------------------------------

TEST_F( WristAnalyzerTest, Analyze_TwoAxisRevoluteWrist )
{
	// Create a joint chain with two orthogonal revolute axes (typical spherical wrist)
	Model::JointChain two_axis_chain( 2 );

	// Joint 1: Rotation around Z-axis
	two_axis_chain.Add(
		Model::Twist( Vec3d( 0, 0, 1 ), Vec3d( 0, 0, 0 ) ),
		Model::Link( Mat4d::Identity(), 0 ),
		Model::Limits( -M_PI, M_PI )
		);

	// Joint 2: Rotation around Y-axis (orthogonal to joint 1)
	two_axis_chain.Add(
		Model::Twist( Vec3d( 0, 1, 0 ), Vec3d( 0, 0, 0 ) ),
		Model::Link( Mat4d::Identity(), 0 ),
		Model::Limits( -M_PI, M_PI )
		);

	Mat4d home_config = ToTransformMatrix( Vec3d( 0.5, 0.0, 0.0 ) );

	auto wrist_group = Model::WristAnalyzer::Analyze( two_axis_chain, home_config );

	EXPECT_TRUE( wrist_group.has_value() );
	EXPECT_EQ( 0, wrist_group->FirstIndex() );
	EXPECT_EQ( 1, wrist_group->LastIndex() );
	EXPECT_EQ( 2, wrist_group->Size() );
	EXPECT_EQ( home_config, wrist_group->tip_home );
}

// ------------------------------------------------------------

TEST_F( WristAnalyzerTest, Analyze_ThreeAxisRevoluteWrist )
{
	// Create a joint chain with three orthogonal revolute axes (typical spherical wrist)
	Model::JointChain three_axis_chain( 3 );

	// Joint 1: Rotation around Z-axis
	three_axis_chain.Add(
		Model::Twist( Vec3d( 0, 0, 1 ), Vec3d( 0, 0, 0 ) ),
		Model::Link( Mat4d::Identity(), 0 ),
		Model::Limits( -M_PI, M_PI )
		);

	// Joint 2: Rotation around Y-axis (orthogonal to joint 1)
	three_axis_chain.Add(
		Model::Twist( Vec3d( 0, 1, 0 ), Vec3d( 0, 0, 0 ) ),
		Model::Link( Mat4d::Identity(), 0 ),
		Model::Limits( -M_PI, M_PI )
		);

	// Joint 3: Rotation around X-axis (orthogonal to joints 1 and 2)
	three_axis_chain.Add(
		Model::Twist( Vec3d( 1, 0, 0 ), Vec3d( 0, 0, 0 ) ),
		Model::Link( Mat4d::Identity(), 0 ),
		Model::Limits( -M_PI, M_PI )
		);

	Mat4d home_config = ToTransformMatrix( Vec3d( 0.5, 0.0, 0.0 ) );

	auto wrist_group = Model::WristAnalyzer::Analyze( three_axis_chain, home_config );

	EXPECT_TRUE( wrist_group.has_value() );
	EXPECT_EQ( 0, wrist_group->FirstIndex() );
	EXPECT_EQ( 2, wrist_group->LastIndex() );
	EXPECT_EQ( 3, wrist_group->Size() );
	EXPECT_EQ( home_config, wrist_group->tip_home );
}

// ------------------------------------------------------------

TEST_F( WristAnalyzerTest, Analyze_FourAxisChain_ThreeAxisRevoluteWrist )
{
	// Create a joint chain with three orthogonal revolute axes (typical spherical wrist)
	Model::JointChain four_axis_chain( 4 );

	// Joint 1: Rotation around Z-axis
	four_axis_chain.Add(
		Model::Twist( Vec3d( 0, 0, 1 ), Vec3d( 0.0, 0, 0 ) ),
		Model::Link( Mat4d::Identity(), 0.5 ),
		Model::Limits( -M_PI, M_PI )
		);

	// Joint 2: Rotation around Y-axis (orthogonal to joint 1)
	four_axis_chain.Add(
		Model::Twist( Vec3d( 0, 1, 0 ), Vec3d( 0.5, 0, 0 ) ),
		Model::Link( ToTransformMatrix( Vec3d( 0.5, 0, 0 ) ), 0 ),
		Model::Limits( -M_PI, M_PI )
		);

	// Joint 3: Rotation around X-axis (orthogonal to joints 1 and 2)
	four_axis_chain.Add(
		Model::Twist( Vec3d( 1, 0, 0 ), Vec3d( 0.5, 0, 0 ) ),
		Model::Link( ToTransformMatrix( Vec3d( 0.5, 0, 0 ) ), 0 ),
		Model::Limits( -M_PI, M_PI )
		);

	// Joint 4: Rotation around Z-axis
	four_axis_chain.Add(
		Model::Twist( Vec3d( 0, 0, 1 ), Vec3d( 0.5, 0, 0 ) ),
		Model::Link( ToTransformMatrix( Vec3d( 0.5, 0, 0 ) ), 0 ),
		Model::Limits( -M_PI, M_PI )
		);

	Mat4d home_config = ToTransformMatrix( Vec3d( 0.5, 0.0, 0.0 ) );

	auto wrist_group = Model::WristAnalyzer::Analyze( four_axis_chain, home_config );

	EXPECT_TRUE( wrist_group.has_value() );
	EXPECT_EQ( 1, wrist_group->FirstIndex() );
	EXPECT_EQ( 3, wrist_group->LastIndex() );
	EXPECT_EQ( 3, wrist_group->Size() );
	EXPECT_TRUE( IsApprox( Mat4d::Identity(), wrist_group->tip_home ) );
}

// ------------------------------------------------------------

TEST_F( WristAnalyzerTest, Analyze_ThreeAxisWrist_WithOffset )
{
	// Create a joint chain with three orthogonal axes but with offset linear components
	Model::JointChain three_axis_chain( 3 );

	// Joint 1: Rotation around Z-axis with offset
	three_axis_chain.Add(
		Model::Twist( Vec3d( 0, 0, 1 ), Vec3d( 0.1, 0.1, 0.1 ) ),
		Model::Link( ToTransformMatrix( Vec3d( 0.1, 0.1, 0.1 ) ), 0 ),
		Model::Limits( -M_PI, M_PI )
		);

	// Joint 2: Rotation around Y-axis with offset
	three_axis_chain.Add(
		Model::Twist( Vec3d( 0, 1, 0 ), Vec3d( 0.1, 0.1, 0.1 ) ),
		Model::Link( ToTransformMatrix( Vec3d( 0.1, 0.1, 0.1 ) ), 0 ),
		Model::Limits( -M_PI, M_PI )
		);

	// Joint 3: Rotation around X-axis with offset
	three_axis_chain.Add(
		Model::Twist( Vec3d( 1, 0, 0 ), Vec3d( 0.1, 0.1, 0.1 ) ),
		Model::Link( ToTransformMatrix( Vec3d( 0.1, 0.1, 0.1 ) ), 0 ),
		Model::Limits( -M_PI, M_PI )
		);

	Mat4d home_config = ToTransformMatrix( Vec3d( 0.5, 0.1, 0.1 ) );

	auto wrist_group = Model::WristAnalyzer::Analyze( three_axis_chain, home_config );

	EXPECT_TRUE( wrist_group.has_value() );
	EXPECT_EQ( 0, wrist_group->FirstIndex() );
	EXPECT_EQ( 2, wrist_group->LastIndex() );
	EXPECT_EQ( 3, wrist_group->Size() );
	EXPECT_TRUE( IsApprox( ToTransformMatrix( Vec3d( 0.4, 0, 0 ) ), wrist_group->tip_home ) );
}

// ------------------------------------------------------------

TEST_F( WristAnalyzerTest, Analyze_ThreeAxisWrist_WithNonOrthogonalAxes )
{
	// Create a joint chain with three non-orthogonal axes
	Model::JointChain non_orthogonal_chain( 3 );

	// Joint 1: Rotation around Z-axis
	non_orthogonal_chain.Add(
		Model::Twist( Vec3d( 0, 0, 1 ), Vec3d( 0, 0, 0 ) ),
		Model::Link( Mat4d::Identity(), 0 ),
		Model::Limits( -M_PI, M_PI )
		);

	// Joint 2: Rotation around an axis close to Y-axis but not exactly orthogonal to Z
	non_orthogonal_chain.Add(
		Model::Twist( Vec3d( 0.1, 1.0, 0.1 ).normalized(), Vec3d( 0, 0, 0 ) ),
		Model::Link( Mat4d::Identity(), 0 ),
		Model::Limits( -M_PI, M_PI )
		);

	// Joint 3: Rotation around an axis close to X-axis but not exactly orthogonal to Z and Y
	non_orthogonal_chain.Add(
		Model::Twist( Vec3d( 1.0, 0.1, 0.1 ).normalized(), Vec3d( 0, 0, 0 ) ),
		Model::Link( Mat4d::Identity(), 0 ),
		Model::Limits( -M_PI, M_PI )
		);

	Mat4d home_config = ToTransformMatrix( Vec3d( 0.5, 0.0, 0.0 ) );

	auto wrist_group = Model::WristAnalyzer::Analyze( non_orthogonal_chain, home_config );

	EXPECT_TRUE( wrist_group.has_value() );
	EXPECT_EQ( 0, wrist_group->FirstIndex() );
	EXPECT_EQ( 2, wrist_group->LastIndex() );
	EXPECT_EQ( 3, wrist_group->Size() );
	EXPECT_EQ( home_config, wrist_group->tip_home );
}

// ------------------------------------------------------------

}
