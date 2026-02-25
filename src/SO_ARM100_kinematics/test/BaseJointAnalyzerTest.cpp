#include "HybridSolver/BaseJointAnalyzer.hpp"

#include "Global.hpp"
#include "HybridSolver/BaseJointModel.hpp"
#include "HybridSolver/WristModel.hpp"
#include "Joint/JointChain.hpp"
#include "Joint/Twist.hpp"
#include "RobotModelTestData.hpp"
#include "Utils/Converter.hpp"

#include <gtest/gtest.h>
#include <optional>

namespace SOArm100::Kinematics::Test
{

// ------------------------------------------------------------
// ------------------------------------------------------------

class BaseJointAnalyzerTest : public ::testing::Test
{
protected:
void SetUp() override
{
	// Create a simple revolute joint chain for testing
	joint_chain_ = Data::GetRevoluteOnlyRobotJointChain();

	// Create a wrist model
	wrist_model_ = WristModel();
	wrist_model_.center_at_home = Vec3d( 1.0, 0.0, 0.0 );
}

void TearDown() override
{
}

JointChain joint_chain_{ 0 };
WristModel wrist_model_{};
};

// ------------------------------------------------------------
// ------------------------------------------------------------

TEST_F( BaseJointAnalyzerTest, Analyze_ValidInput )
{
	BaseJointAnalyzer analyzer;
	auto result = analyzer.Analyze( joint_chain_, wrist_model_ );

	// Check that a valid model was returned
	ASSERT_TRUE( result.has_value() ) << "Should return a valid BaseJointModel";

	// Check the reference direction
	// Expected reference direction is the projection of wrist center onto the plane orthogonal to the base joint axis
	const Vec3d expected_reference_direction = Vec3d( 1, 0, 0 );

	EXPECT_TRUE( result->reference_direction.isApprox( expected_reference_direction, 1e-6 ) )
	    << "Reference direction should match expected value";
}

// ------------------------------------------------------------

TEST_F( BaseJointAnalyzerTest, Analyze_NoWristModel )
{
	BaseJointAnalyzer analyzer;
	auto result = analyzer.Analyze( joint_chain_, std::nullopt );

	// Check that no model was returned
	EXPECT_FALSE( result.has_value() ) << "Should return std::nullopt when no wrist model is provided";
}

// ------------------------------------------------------------

TEST_F( BaseJointAnalyzerTest, Analyze_EmptyJointChain )
{
	BaseJointAnalyzer analyzer;
	JointChain empty_chain( 0 );

	auto result = analyzer.Analyze( empty_chain, wrist_model_ );

	// Check that no model was returned
	EXPECT_FALSE( result.has_value() ) << "Should return std::nullopt when joint chain is empty";
}

// ------------------------------------------------------------

TEST_F( BaseJointAnalyzerTest, Analyze_WithDifferentWristPositions )
{
	BaseJointAnalyzer analyzer;

	// Test with wrist center at origin
	WristModel wrist_model_origin;
	wrist_model_origin.center_at_home = Vec3d( 0.0, 1.0, 0.0 );

	auto result_origin = analyzer.Analyze( joint_chain_, wrist_model_origin );
	ASSERT_TRUE( result_origin.has_value() );

	// The reference direction should be orthogonal unit since the wrist center is on the axis
	auto base_twist = joint_chain_.GetActiveJointTwist(0);
	EXPECT_EQ( result_origin->reference_direction.dot( base_twist.GetAxis() ), 0.0 )
	    << "Reference direction should be zero when wrist center is on the axis";

	// Test with wrist center at (1, 0, 0)
	WristModel wrist_model_x;
	wrist_model_x.center_at_home = Vec3d( 1.0, 0.0, 0.0 );

	auto result_x = analyzer.Analyze( joint_chain_, wrist_model_x );
	ASSERT_TRUE( result_x.has_value() );

	// Expected
	// reference direction is (1, 0, 0) since it's already orthogonal to the Z-axis
	EXPECT_TRUE( result_x->reference_direction.isApprox( Vec3d( 1.0, 0.0, 0.0 ), epsilon ) )
	    << "Reference direction should match expected value for wrist at (1, 0, 0)";
}

// ------------------------------------------------------------

TEST_F( BaseJointAnalyzerTest, Analyze_WithDifferentBaseJointAxis )
{
	// Create a joint chain with a different base joint axis
	JointChain joint_chain_y( 2 );
	Vec3d origin = Vec3d::Zero();
	Mat4d origin_transform = Mat4d::Identity();
	Vec3d axis = Vec3d::UnitY(); // Rotation around Y-axis
	Twist twist_y( axis, origin ); 
	Link link_y( origin_transform );
	Limits limits_y( -M_PI, M_PI );
	joint_chain_y.Add( twist_y, link_y, limits_y );

	origin << 0,0,1;
	origin_transform = ToTransformMatrix( origin );
	axis = ( origin_transform * Vec3d::UnitZ().homogeneous() ).head( 3 ); // Rotation around Z-axis
	Twist twist_z( axis, origin ); 
	Link link_z( origin_transform );
	Limits limits_z( -M_PI, M_PI );
	joint_chain_y.Add( twist_z, link_z, limits_z );

	BaseJointAnalyzer analyzer;

	// Test with wrist center at (0, 0, 1)
	WristModel wrist_model_z;
	wrist_model_z.active_joint_start = 1;
	wrist_model_z.active_joint_count = 1;
	wrist_model_z.center_at_home = Vec3d( 0.0, 0.0, 1.0 );

	auto result = analyzer.Analyze( joint_chain_y, wrist_model_z );
	ASSERT_TRUE( result.has_value() );

	// Expected reference direction is (0, 0, 1) since it's already orthogonal to the Y-axis
	Vec3d expected_reference_direction = Vec3d::UnitZ();
	EXPECT_TRUE( result->reference_direction.isApprox( expected_reference_direction, epsilon ) )
	    << "Expected Reference direction= " << expected_reference_direction.transpose() << std::endl
		<< "Result Reference direction  = " << result->reference_direction.transpose() << std::endl;
}

// ------------------------------------------------------------

TEST_F( BaseJointAnalyzerTest, Analyze_ReferenceDirectionNormalization )
{
	BaseJointAnalyzer analyzer;

	// Test with a wrist center that results in a non-unit reference direction
	WristModel wrist_model;
	wrist_model.center_at_home = Vec3d( 2.0, 0.0, 0.0 ); // Further from the axis

	auto result = analyzer.Analyze( joint_chain_, wrist_model );
	ASSERT_TRUE( result.has_value() );

	// The reference direction should be a unit vector
	double norm = result->reference_direction.norm();
	EXPECT_NEAR( norm, 1.0, epsilon ) << "Reference direction should be a unit vector";
}

// ------------------------------------------------------------

}
