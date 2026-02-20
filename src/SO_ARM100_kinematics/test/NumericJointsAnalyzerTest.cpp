#include "Global.hpp"

#include "HybridSolver/NumericJointsAnalyzer.hpp"

#include "HybridSolver/BaseJointModel.hpp"
#include "HybridSolver/NumericJointsModel.hpp"
#include "HybridSolver/WristModel.hpp"
#include "Joint/JointChain.hpp"
#include "Joint/Twist.hpp"
#include "Utils/Converter.hpp"
#include "Utils/KinematicsUtils.hpp"
#include "RobotModelTestData.hpp"

#include <gtest/gtest.h>
#include <optional>

namespace SOArm100::Kinematics::Test
{

// ------------------------------------------------------------
// ------------------------------------------------------------

class NumericJointsAnalyzerTest : public ::testing::Test
{
protected:
void SetUp() override
{
	// Create a simple revolute joint chain for testing
	joint_chain_ = Data::GetRevoluteOnlyRobotJointChain();
	home_configuration_ = Mat4d::Identity();

	// Create a base joint model
	base_joint_model_.reference_direction = Vec3d( 1.0, 0.0, 0.0 );

	// Create a wrist model for the last 3 joints
	wrist_model_.type = WristType::Revolute1;
	wrist_model_.active_joint_start = joint_chain_.GetActiveJointCount() - 1;
	wrist_model_.active_joint_count = 1;
	wrist_model_.center_at_home = Vec3d( 1.0, 0, 0 );
	wrist_model_.tcp_in_wrist_at_home = Mat4d::Identity();
	wrist_model_.tcp_in_wrist_at_home_inv = Mat4d::Identity();
}

void TearDown() override
{
}

JointChain joint_chain_{ 0 };
Mat4d home_configuration_{ Mat4d::Identity() };
BaseJointModel base_joint_model_{};
WristModel wrist_model_{};
};

// ------------------------------------------------------------

TEST_F( NumericJointsAnalyzerTest, Analyze_FullNumericChain )
{
	NumericJointsAnalyzer analyzer;

	// Test with no base joint and no wrist model (all joints are numeric)
	auto result = analyzer.Analyze( joint_chain_, home_configuration_, std::nullopt, std::nullopt );

	// Check that a valid model was returned
	ASSERT_TRUE( result.has_value() ) << "Should return a valid NumericJointsModel";

	// Check the numeric joint model properties
	EXPECT_EQ( result->start_index, 0 ) << "Start index should be 0";
	EXPECT_EQ( result->count, joint_chain_.GetActiveJointCount() ) << "Count should match total active joints";
	EXPECT_TRUE( result->home_configuration.isApprox( home_configuration_, 1e-6 ) )
	    << "Home configuration should match input";
}

// ------------------------------------------------------------

TEST_F( NumericJointsAnalyzerTest, Analyze_WithBaseJoint )
{
	NumericJointsAnalyzer analyzer;

	// Test with a base joint but no wrist model
	auto result = analyzer.Analyze( joint_chain_, home_configuration_, base_joint_model_, std::nullopt );

	// Check that a valid model was returned
	ASSERT_TRUE( result.has_value() ) << "Should return a valid NumericJointsModel";

	// Check the numeric joint model properties
	EXPECT_EQ( result->start_index, 0 ) << "Start index should be 0";
	EXPECT_EQ( result->count, joint_chain_.GetActiveJointCount() ) << "Count should be total active joints";
	EXPECT_TRUE( result->home_configuration.isApprox( home_configuration_, 1e-6 ) )
	    << "Home configuration should match input";
}

// ------------------------------------------------------------

TEST_F( NumericJointsAnalyzerTest, Analyze_WithWristModel )
{
	NumericJointsAnalyzer analyzer;

	// Test with no base joint but with a wrist model
	auto result = analyzer.Analyze( joint_chain_, home_configuration_, std::nullopt, wrist_model_ );

	// Check that a valid model was returned
	ASSERT_TRUE( result.has_value() ) << "Should return a valid NumericJointsModel";

	// Check the numeric joint model properties
	EXPECT_EQ( result->start_index, 0 ) << "Start index should be 0";
	EXPECT_EQ( result->count, joint_chain_.GetActiveJointCount() - wrist_model_.active_joint_count )
	    << "Count should be total active joints minus wrist joints";
	EXPECT_TRUE( result->home_configuration.isApprox( home_configuration_, 1e-6 ) )
	    << "Home configuration should match input";
}

// ------------------------------------------------------------

TEST_F( NumericJointsAnalyzerTest, Analyze_WithBaseJointAndWristModel )
{
	NumericJointsAnalyzer analyzer;

	// Test with both a base joint and a wrist model
	auto result = analyzer.Analyze( joint_chain_, home_configuration_, base_joint_model_, wrist_model_ );

	// Check that a valid model was returned
	ASSERT_TRUE( result.has_value() ) << "Should return a valid NumericJointsModel";

	// Check the numeric joint model properties
	EXPECT_EQ( result->start_index, 0 ) << "Start index should be 0";
	EXPECT_EQ( result->count, joint_chain_.GetActiveJointCount() - wrist_model_.active_joint_count )
	    << "Count should be total active joints wrist joints";
	EXPECT_TRUE( result->home_configuration.isApprox( home_configuration_, 1e-6 ) )
	    << "Home configuration should match input";
}

// ------------------------------------------------------------

TEST_F( NumericJointsAnalyzerTest, Analyze_NoNumericJoints )
{
	NumericJointsAnalyzer analyzer;

	// Create a joint chain with only 3 joints (all used by wrist)
	JointChain wrist_only_chain( 3 );
	wrist_only_chain.Add(
		Twist( Vec3d( 0, 0, 1 ), Vec3d( 0, 0, 0 ) ),
		Link( Mat4d::Identity() ),
		Limits( -M_PI, M_PI )
		);
	wrist_only_chain.Add(
		Twist( Vec3d( 0, 1, 0 ), Vec3d( 0, 0, 0 ) ),
		Link( Mat4d::Identity() ),
		Limits( -M_PI, M_PI )
		);
	wrist_only_chain.Add(
		Twist( Vec3d( 1, 0, 0 ), Vec3d( 0, 0, 0 ) ),
		Link( Mat4d::Identity() ),
		Limits( -M_PI, M_PI )
		);

	// Create a wrist model for all joints
	WristModel full_wrist_model;
	full_wrist_model.type = WristType::Revolute3;
	full_wrist_model.active_joint_start = 0;
	full_wrist_model.active_joint_count = 3;
	full_wrist_model.center_at_home = Vec3d( 0, 0, 0 );
	full_wrist_model.tcp_in_wrist_at_home = Mat4d::Identity();
	full_wrist_model.tcp_in_wrist_at_home_inv = Mat4d::Identity();

	// Test with a base joint and a wrist model that covers all joints
	auto result = analyzer.Analyze( wrist_only_chain, home_configuration_, base_joint_model_, full_wrist_model );

	// Check that no model was returned (no numeric joints left)
	EXPECT_FALSE( result.has_value() ) << "Should return std::nullopt when no numeric joints are left";
}

// ------------------------------------------------------------

TEST_F( NumericJointsAnalyzerTest, Analyze_WithNonIdentityHomeConfiguration )
{
	NumericJointsAnalyzer analyzer;

	// Create a non-identity home configuration
	Mat4d non_identity_home = Mat4d::Identity();
	non_identity_home.block< 3, 3 >( 0, 0 ) = Eigen::AngleAxisd( M_PI / 4, Vec3d( 0, 0, 1 ) ).toRotationMatrix();
	non_identity_home.block< 3, 1 >( 0, 3 ) = Vec3d( 0.5, 0.0, 0.0 );

	// Test with a base joint and a wrist model
	auto result = analyzer.Analyze( joint_chain_, non_identity_home, base_joint_model_, wrist_model_ );

	// Check that a valid model was returned
	ASSERT_TRUE( result.has_value() ) << "Should return a valid NumericJointsModel";

	// Check that the home configuration is correctly set
	EXPECT_TRUE( result->home_configuration.isApprox( non_identity_home, 1e-6 ) )
	    << "Home configuration should match input";
}

// ------------------------------------------------------------

TEST_F( NumericJointsAnalyzerTest, Analyze_WithReducedHomeConfiguration )
{
	NumericJointsAnalyzer analyzer;

	// Create a home configuration with translation
	Mat4d full_home = Mat4d::Identity();
	full_home.block< 3, 1 >( 0, 3 ) = Vec3d( 1.0, 0.0, 0.5 );

	// Create a wrist model with a non-identity TCP transform
	WristModel wrist_model;
	wrist_model.type = WristType::Revolute1;
	wrist_model.active_joint_start = joint_chain_.GetActiveJointCount() - 1;
	wrist_model.active_joint_count = 1;
	wrist_model.center_at_home = Vec3d( 1.0, 0, 0 );
	wrist_model.tcp_in_wrist_at_home = ToTransformMatrix( Vec3d( 0, 0, 0.5 ) );
	wrist_model.tcp_in_wrist_at_home_inv = Inverse( wrist_model.tcp_in_wrist_at_home );

	// Test with a base joint and a wrist model
	auto result = analyzer.Analyze( joint_chain_, full_home, base_joint_model_, wrist_model );

	// Check that a valid model was returned
	ASSERT_TRUE( result.has_value() ) << "Should return a valid NumericJointsModel";

	// Check that the home configuration is correctly reduced
	Mat4d expected_reduced_home = full_home * wrist_model.tcp_in_wrist_at_home_inv;
	EXPECT_TRUE( result->home_configuration.isApprox( expected_reduced_home, 1e-6 ) )
	    << "Expected reduced home=\n" << expected_reduced_home << std::endl
	    << "Result reduced home=\n" << result->home_configuration << std::endl;
}

// ------------------------------------------------------------

} // namespace SOArm100::Kinematics::Test
