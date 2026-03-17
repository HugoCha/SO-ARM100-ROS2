#include "HybridSolver/WristAnalyzer.hpp"

#include "Global.hpp"
#include "HybridSolver/WristModel.hpp"
#include "Model/JointChain.hpp"
#include "Model/Twist.hpp"
#include "RobotModelTestData.hpp"
#include "Utils/Converter.hpp"

#include <gtest/gtest.h>

namespace SOArm100::Kinematics::Test
{

// ------------------------------------------------------------
// ------------------------------------------------------------

class WristAnalyzerTest : public ::testing::Test
{
protected:
void SetUp() override
{
	// Create a simple revolute joint chain for testing
	joint_chain_ = Data::GetRevoluteOnlyRobotJointChain();
	home_configuration_ = Mat4d::Identity();
}

void TearDown() override
{
}

JointChain joint_chain_{ 0 };
Mat4d home_configuration_{ Mat4d::Identity() };
};

// ------------------------------------------------------------
// ------------------------------------------------------------

TEST_F( WristAnalyzerTest, Analyze_OneAxisRevoluteWrist )
{
	WristAnalyzer analyzer;

	// Create a joint chain with one revolute axis (typical spherical wrist)
	JointChain one_axis_chain( 1 );

	// Joint 1: Rotation around Z-axis
	one_axis_chain.Add(
		Twist( Vec3d( 0, 0, 1 ), Vec3d( 0, 0, 0 ) ),
		Link( Mat4d::Identity(), 0 ),
		Limits( -M_PI, M_PI )
		);

	// Set a home configuration with some translation
	Mat4d home_config = Mat4d::Identity();
	home_config.block< 3, 1 >( 0, 3 ) = Vec3d( 0.5, 0.0, 0.0 );

	auto result = analyzer.Analyze( one_axis_chain, home_config );

	// Check that a valid model was returned
	ASSERT_TRUE( result.has_value() ) << "Should return a valid WristModel for 1-axis revolute wrist";

	// Check the wrist model properties
	EXPECT_EQ( result->type, WristType::Revolute1 ) << "Wrist type should be Revolute3";
	EXPECT_EQ( result->active_joint_start, 0 ) << "Active joint start should be 0";
	EXPECT_EQ( result->active_joint_count, 1 ) << "Active joint count should be 1";

	// Check that the intersection point is at the origin (since all joints intersect at origin)
	EXPECT_TRUE( result->center_at_home.isApprox( Vec3d( 0, 0, 0 ), 1e-6 ) )
	    << "Intersection point should be at the origin for this configuration";

	// Check that the TCP in wrist transform is valid
	// Expected: translation should be 0.5 in X (since home config is at 0.5,0,0 and wrist center is at 0,0,0)
	Mat4d expected_tcp = Mat4d::Identity();
	expected_tcp.block< 3, 1 >( 0, 3 ) = Vec3d( 0.5, 0.0, 0.0 );

	EXPECT_TRUE( result->tcp_in_wrist_at_home.isApprox( expected_tcp, 1e-6 ) )
	    << "Expected TCP/wrist=\n" << expected_tcp
	    << "\nTCP/Wrist=\n" << result->tcp_in_wrist_at_home << std::endl;

	// Check that the inverse TCP transform is correct
	Mat4d expected_inverse_tcp = Mat4d::Identity();
	expected_inverse_tcp.block< 3, 1 >( 0, 3 ) = Vec3d( -0.5, 0.0, 0.0 );

	EXPECT_TRUE( result->tcp_in_wrist_at_home_inv.isApprox( expected_inverse_tcp, 1e-6 ) )
	    << "Expected TCP/wrist inv=\n" << expected_inverse_tcp
	    << "\nTCP/Wrist inv=\n" << result->tcp_in_wrist_at_home_inv << std::endl;
}

// ------------------------------------------------------------

TEST_F( WristAnalyzerTest, Analyze_TwoAxisRevoluteWrist )
{
	WristAnalyzer analyzer;

	// Create a joint chain with two orthogonal revolute axes (typical spherical wrist)
	JointChain two_axis_chain( 2 );

	// Joint 1: Rotation around Z-axis
	two_axis_chain.Add(
		Twist( Vec3d( 0, 0, 1 ), Vec3d( 0, 0, 0 ) ),
		Link( Mat4d::Identity(), 0 ),
		Limits( -M_PI, M_PI )
		);

	// Joint 2: Rotation around Y-axis (orthogonal to joint 1)
	two_axis_chain.Add(
		Twist( Vec3d( 0, 1, 0 ), Vec3d( 0, 0, 0 ) ),
		Link( Mat4d::Identity(), 0 ),
		Limits( -M_PI, M_PI )
		);

	// Set a home configuration with some translation
	Mat4d home_config = Mat4d::Identity();
	home_config.block< 3, 1 >( 0, 3 ) = Vec3d( 0.5, 0.0, 0.0 );

	auto result = analyzer.Analyze( two_axis_chain, home_config );

	// Check that a valid model was returned
	ASSERT_TRUE( result.has_value() ) << "Should return a valid WristModel for 2-axis revolute wrist";

	// Check the wrist model properties
	EXPECT_EQ( result->type, WristType::Revolute2 ) << "Wrist type should be Revolute3";
	EXPECT_EQ( result->active_joint_start, 0 ) << "Active joint start should be 0";
	EXPECT_EQ( result->active_joint_count, 2 ) << "Active joint count should be 2";

	// Check that the intersection point is at the origin (since all joints intersect at origin)
	EXPECT_TRUE( result->center_at_home.isApprox( Vec3d( 0, 0, 0 ), 1e-6 ) )
	    << "Intersection point should be at the origin for this configuration";

	// Check that the TCP in wrist transform is valid
	// Expected: translation should be 0.5 in X (since home config is at 0.5,0,0 and wrist center is at 0,0,0)
	Mat4d expected_tcp = Mat4d::Identity();
	expected_tcp.block< 3, 1 >( 0, 3 ) = Vec3d( 0.5, 0.0, 0.0 );

	EXPECT_TRUE( result->tcp_in_wrist_at_home.isApprox( expected_tcp, 1e-6 ) )
	    << "Expected TCP/wrist=\n" << expected_tcp
	    << "\nTCP/Wrist=\n" << result->tcp_in_wrist_at_home << std::endl;

	// Check that the inverse TCP transform is correct
	Mat4d expected_inverse_tcp = Mat4d::Identity();
	expected_inverse_tcp.block< 3, 1 >( 0, 3 ) = Vec3d( -0.5, 0.0, 0.0 );

	EXPECT_TRUE( result->tcp_in_wrist_at_home_inv.isApprox( expected_inverse_tcp, 1e-6 ) )
	    << "Expected TCP/wrist inv=\n" << expected_inverse_tcp
	    << "\nTCP/Wrist inv=\n" << result->tcp_in_wrist_at_home_inv << std::endl;
}

// ------------------------------------------------------------

TEST_F( WristAnalyzerTest, Analyze_ThreeAxisRevoluteWrist )
{
	WristAnalyzer analyzer;

	// Create a joint chain with three orthogonal revolute axes (typical spherical wrist)
	JointChain three_axis_chain( 3 );

	// Joint 1: Rotation around Z-axis
	three_axis_chain.Add(
		Twist( Vec3d( 0, 0, 1 ), Vec3d( 0, 0, 0 ) ),
		Link( Mat4d::Identity(), 0 ),
		Limits( -M_PI, M_PI )
		);

	// Joint 2: Rotation around Y-axis (orthogonal to joint 1)
	three_axis_chain.Add(
		Twist( Vec3d( 0, 1, 0 ), Vec3d( 0, 0, 0 ) ),
		Link( Mat4d::Identity(), 0 ),
		Limits( -M_PI, M_PI )
		);

	// Joint 3: Rotation around X-axis (orthogonal to joints 1 and 2)
	three_axis_chain.Add(
		Twist( Vec3d( 1, 0, 0 ), Vec3d( 0, 0, 0 ) ),
		Link( Mat4d::Identity(), 0 ),
		Limits( -M_PI, M_PI )
		);

	// Set a home configuration with some translation
	Mat4d home_config = Mat4d::Identity();
	home_config.block< 3, 1 >( 0, 3 ) = Vec3d( 0.5, 0.0, 0.0 );

	auto result = analyzer.Analyze( three_axis_chain, home_config );

	// Check that a valid model was returned
	ASSERT_TRUE( result.has_value() ) << "Should return a valid WristModel for 3-axis revolute wrist";

	// Check the wrist model properties
	EXPECT_EQ( result->type, WristType::Revolute3 ) << "Wrist type should be Revolute3";
	EXPECT_EQ( result->active_joint_start, 0 ) << "Active joint start should be 0";
	EXPECT_EQ( result->active_joint_count, 3 ) << "Active joint count should be 3";

	// Check that the intersection point is at the origin (since all joints intersect at origin)
	EXPECT_TRUE( result->center_at_home.isApprox( Vec3d( 0, 0, 0 ), 1e-6 ) )
	    << "Intersection point should be at the origin for this configuration";

	// Check that the TCP in wrist transform is valid
	// Expected: translation should be 0.5 in X (since home config is at 0.5,0,0 and wrist center is at 0,0,0)
	Mat4d expected_tcp = Mat4d::Identity();
	expected_tcp.block< 3, 1 >( 0, 3 ) = Vec3d( 0.5, 0.0, 0.0 );

	EXPECT_TRUE( result->tcp_in_wrist_at_home.isApprox( expected_tcp, 1e-6 ) )
	    << "Expected TCP/wrist=\n" << expected_tcp
	    << "\nTCP/Wrist=\n" << result->tcp_in_wrist_at_home << std::endl;

	// Check that the inverse TCP transform is correct
	Mat4d expected_inverse_tcp = Mat4d::Identity();
	expected_inverse_tcp.block< 3, 1 >( 0, 3 ) = Vec3d( -0.5, 0.0, 0.0 );

	EXPECT_TRUE( result->tcp_in_wrist_at_home_inv.isApprox( expected_inverse_tcp, 1e-6 ) )
	    << "Expected TCP/wrist inv=\n" << expected_inverse_tcp
	    << "\nTCP/Wrist inv=\n" << result->tcp_in_wrist_at_home_inv << std::endl;
}

// ------------------------------------------------------------

TEST_F( WristAnalyzerTest, Analyze_FourAxisChain_ThreeAxisRevoluteWrist )
{
	WristAnalyzer analyzer;

	// Create a joint chain with three orthogonal revolute axes (typical spherical wrist)
	JointChain four_axis_chain( 4 );

	// Joint 1: Rotation around Z-axis
	four_axis_chain.Add(
		Twist( Vec3d( 0, 0, 1 ), Vec3d( 0.0, 0, 0 ) ),
		Link( Mat4d::Identity(), 0 ),
		Limits( -M_PI, M_PI )
		);

	// Joint 2: Rotation around Y-axis (orthogonal to joint 1)
	four_axis_chain.Add(
		Twist( Vec3d( 0, 1, 0 ), Vec3d( 0.5, 0, 0 ) ),
		Link( Mat4d::Identity(), 0 ),
		Limits( -M_PI, M_PI )
		);

	// Joint 3: Rotation around X-axis (orthogonal to joints 1 and 2)
	four_axis_chain.Add(
		Twist( Vec3d( 1, 0, 0 ), Vec3d( 0.5, 0, 0 ) ),
		Link( Mat4d::Identity(), 0 ),
		Limits( -M_PI, M_PI )
		);

	// Joint 4: Rotation around Z-axis
	four_axis_chain.Add(
		Twist( Vec3d( 0, 0, 1 ), Vec3d( 0.5, 0, 0 ) ),
		Link( Mat4d::Identity(), 0 ),
		Limits( -M_PI, M_PI )
		);

	// Set a home configuration with some translation
	Mat4d home_config = Mat4d::Identity();
	home_config.block< 3, 1 >( 0, 3 ) = Vec3d( 0.5, 0.0, 0.0 );

	auto result = analyzer.Analyze( four_axis_chain, home_config );

	// Check that a valid model was returned
	ASSERT_TRUE( result.has_value() ) << "Should return a valid WristModel for 3-axis revolute wrist";

	// Check the wrist model properties
	EXPECT_EQ( result->type, WristType::Revolute3 ) << "Wrist type should be Revolute3";
	EXPECT_EQ( result->active_joint_start, 1 ) << "Active joint start should be 1";
	EXPECT_EQ( result->active_joint_count, 3 ) << "Active joint count should be 3";

	// Check that the intersection point is at the origin (since all joints intersect at origin)
	EXPECT_TRUE( result->center_at_home.isApprox( Vec3d( 0.5, 0, 0 ), 1e-6 ) )
	    << "Intersection point should be at the origin for this configuration";

	// Check that the TCP in wrist transform is valid
	// Expected: translation should be 0.5 in X (since home config is at 0.5,0,0 and wrist center is at 0,0,0)
	Mat4d expected_tcp = Mat4d::Identity();

	EXPECT_TRUE( result->tcp_in_wrist_at_home.isApprox( expected_tcp, 1e-6 ) )
	    << "Expected TCP/wrist=\n" << expected_tcp
	    << "\nTCP/Wrist=\n" << result->tcp_in_wrist_at_home << std::endl;

	// Check that the inverse TCP transform is correct
	Mat4d expected_inverse_tcp = Mat4d::Identity();

	EXPECT_TRUE( result->tcp_in_wrist_at_home_inv.isApprox( expected_inverse_tcp, 1e-6 ) )
	    << "Expected TCP/wrist inv=\n" << expected_inverse_tcp
	    << "\nTCP/Wrist inv=\n" << result->tcp_in_wrist_at_home_inv << std::endl;
}

// ------------------------------------------------------------

TEST_F( WristAnalyzerTest, Analyze_ThreeAxisWrist_WithOffset )
{
	WristAnalyzer analyzer;

	// Create a joint chain with three orthogonal axes but with offset linear components
	JointChain three_axis_chain( 3 );

	// Joint 1: Rotation around Z-axis with offset
	three_axis_chain.Add(
		Twist( Vec3d( 0, 0, 1 ), Vec3d( 0.1, 0.1, 0.1 ) ),
		Link( ToTransformMatrix( Vec3d(0.1, 0.1, 0.1 ) ), 0 ),
		Limits( -M_PI, M_PI )
		);

	// Joint 2: Rotation around Y-axis with offset
	three_axis_chain.Add(
		Twist( Vec3d( 0, 1, 0 ), Vec3d( 0.1, 0.1, 0.1 ) ),
		Link( ToTransformMatrix( Vec3d(0.1, 0.1, 0.1 ) ), 0 ),
		Limits( -M_PI, M_PI )
		);

	// Joint 3: Rotation around X-axis with offset
	three_axis_chain.Add(
		Twist( Vec3d( 1, 0, 0 ), Vec3d( 0.1, 0.1, 0.1 ) ),
		Link( ToTransformMatrix( Vec3d(0.1, 0.1, 0.1 ) ), 0 ),
		Limits( -M_PI, M_PI )
		);

	// Set a home configuration
	Mat4d home_config = Mat4d::Identity();
	home_config.block< 3, 1 >( 0, 3 ) = Vec3d( 0.5, 0.0, 0.0 );

	auto result = analyzer.Analyze( three_axis_chain, home_config );

	// Check that a valid model was returned
	ASSERT_TRUE( result.has_value() ) << "Should return a valid WristModel for 3-axis wrist with offsets";

	// Check the wrist model properties
	EXPECT_EQ( result->type, WristType::Revolute3 ) << "Wrist type should be Revolute3";
	EXPECT_EQ( result->active_joint_start, 0 ) << "Active joint start should be 0";
	EXPECT_EQ( result->active_joint_count, 3 ) << "Active joint count should be 3";

	// Check that the intersection point is not at the origin (since joints have offsets)
	EXPECT_FALSE( result->center_at_home.isApprox( Vec3d( 0, 0, 0 ), 1e-6 ) )
	    << "Intersection point should not be at the origin for this configuration with offsets";
}

// ------------------------------------------------------------

TEST_F( WristAnalyzerTest, Analyze_OneAxisWrist_WithNonOrthogonalAxes )
{
	WristAnalyzer analyzer;

	// Create a joint chain with three non-orthogonal axes
	JointChain non_orthogonal_chain( 3 );

	// Joint 1: Rotation around Z-axis
	non_orthogonal_chain.Add(
		Twist( Vec3d( 0, 0, 1 ), Vec3d( 0, 0, 0 ) ),
		Link( Mat4d::Identity(), 0 ),
		Limits( -M_PI, M_PI )
		);

	// Joint 2: Rotation around an axis close to Y-axis but not exactly orthogonal to Z
	non_orthogonal_chain.Add(
		Twist( Vec3d( 0.1, 1.0, 0.1 ).normalized(), Vec3d( 0, 0, 0 ) ),
		Link( Mat4d::Identity(), 0 ),
		Limits( -M_PI, M_PI )
		);

	// Joint 3: Rotation around an axis close to X-axis but not exactly orthogonal to Z and Y
	non_orthogonal_chain.Add(
		Twist( Vec3d( 1.0, 0.1, 0.1 ).normalized(), Vec3d( 0, 0, 0 ) ),
		Link( Mat4d::Identity(), 0 ),
		Limits( -M_PI, M_PI )
		);

	auto result = analyzer.Analyze( non_orthogonal_chain, home_configuration_ );
	// Check that a valid model was returned

	ASSERT_TRUE( result.has_value() ) << "Should return a valid WristModel for 3-axis wrist with offsets";

	// Check the wrist model properties
	EXPECT_EQ( result->type, WristType::Revolute3 ) << "Wrist type should be Revolute3";
	EXPECT_EQ( result->active_joint_start, 0 ) << "Active joint start should be 0";
	EXPECT_EQ( result->active_joint_count, 3 ) << "Active joint count should be 3";
}

// ------------------------------------------------------------

TEST_F( WristAnalyzerTest, Analyze_ThreeAxisWrist_WithDifferentHomeConfigurations )
{
	WristAnalyzer analyzer;

	// Create a joint chain with three orthogonal axes
	JointChain three_axis_chain( 3 );

	// Joint 1: Rotation around Z-axis
	three_axis_chain.Add(
		Twist( Vec3d( 0, 0, 1 ), Vec3d( 0, 0, 0 ) ),
		Link( Mat4d::Identity(), 0 ),
		Limits( -M_PI, M_PI )
		);

	// Joint 2: Rotation around Y-axis
	three_axis_chain.Add(
		Twist( Vec3d( 0, 1, 0 ), Vec3d( 0, 0, 0 ) ),
		Link( Mat4d::Identity(), 0 ),
		Limits( -M_PI, M_PI )
		);

	// Joint 3: Rotation around X-axis
	three_axis_chain.Add(
		Twist( Vec3d( 1, 0, 0 ), Vec3d( 0, 0, 0 ) ),
		Link( Mat4d::Identity(), 0 ),
		Limits( -M_PI, M_PI )
		);

	// Test with different home configurations
	std::vector< Mat4d > home_configurations = {
		Mat4d::Identity(),  // No translation
		[](){
			Mat4d m = Mat4d::Identity(); m.block< 3, 1 >( 0, 3 ) = Vec3d( 0.5, 0.0, 0.0 ); return m;
		}(),                                                                                            // Translation in X
		[](){
			Mat4d m = Mat4d::Identity(); m.block< 3, 1 >( 0, 3 ) = Vec3d( 0.0, 0.5, 0.0 ); return m;
		}(),                                                                                            // Translation in Y
		[](){
			Mat4d m = Mat4d::Identity(); m.block< 3, 1 >( 0, 3 ) = Vec3d( 0.0, 0.0, 0.5 ); return m;
		}()                                                                                             // Translation in Z
	};

	for ( const auto& home_config : home_configurations )
	{
		auto result = analyzer.Analyze( three_axis_chain, home_config );

		// Check that a valid model was returned for each configuration
		ASSERT_TRUE( result.has_value() ) << "Should return a valid WristModel for each home configuration";

		// Check the wrist model properties
		EXPECT_EQ( result->type, WristType::Revolute3 ) << "Wrist type should be Revolute3";
		EXPECT_EQ( result->active_joint_start, 0 ) << "Active joint start should be 0";
		EXPECT_EQ( result->active_joint_count, 3 ) << "Active joint count should be 3";

		// Check that the intersection point is at the origin
		EXPECT_TRUE( result->center_at_home.isApprox( Vec3d( 0, 0, 0 ), 1e-6 ) )
		    << "Intersection point should be at the origin for this configuration";

		// Check that the TCP in wrist transform is valid
		Mat4d expected_tcp = Mat4d::Identity();
		expected_tcp.block< 3, 1 >( 0, 3 ) = home_config.block< 3, 1 >( 0, 3 );

		EXPECT_TRUE( result->tcp_in_wrist_at_home.isApprox( expected_tcp, 1e-6 ) )
		    << "TCP in wrist at home should match expected transform for this home configuration";
	}
}

// ------------------------------------------------------------

} // namespace SOArm100::Kinematics::Test
