#include "HybridSolver/HybridSolverAnalyzer.hpp"

#include "Global.hpp"
#include "Joint/JointChain.hpp"
#include "Joint/Twist.hpp"
#include "Joint/Link.hpp"
#include "Joint/Limits.hpp"
#include "HybridSolver/HybridSolverConfiguration.hpp"
#include "Utils/Converter.hpp"

#include <gtest/gtest.h>
#include <memory>

namespace SOArm100::Kinematics::Test
{

// ------------------------------------------------------------
// Helper function to create a test joint chain with base, numeric, and wrist joints
// ------------------------------------------------------------

std::shared_ptr< JointChain > CreateTestJointChain()
{
	// Create a joint chain with 6 joints: 1 base joint + 3 numeric joints + 2 wrist joints
	auto joint_chain = std::make_shared< JointChain >( 6 );

	// Base joint (revolute around Z-axis)
	joint_chain->Add(
		Twist( Vec3d( 0, 0, 1 ), Vec3d( 0, 0, 0 ) ),
		Link( Mat4d::Identity() ),
		Limits( -M_PI, M_PI )
		);

	// Numeric joints (3 joints)
	joint_chain->Add(
		Twist( Vec3d( 0, 1, 0 ), Vec3d( 0, 0, 0.5 ) ),
		Link( Mat4d::Identity() ),
		Limits( -M_PI / 2, M_PI / 2 )
		);
	joint_chain->Add(
		Twist( Vec3d( 0, 1, 0 ), Vec3d( 0, 0, 1.0 ) ),
		Link( Mat4d::Identity() ),
		Limits( -M_PI / 2, M_PI / 2 )
		);
	joint_chain->Add(
		Twist( Vec3d( 0, 0, 1 ), Vec3d( 0, 0, 1.5 ) ),
		Link( Mat4d::Identity() ),
		Limits( -M_PI, M_PI )
		);

	// Wrist joints (2 joints)
	joint_chain->Add(
		Twist( Vec3d( 1, 0, 0 ), Vec3d( 0, 0, 1.5 ) ),
		Link( Mat4d::Identity() ),
		Limits( -M_PI, M_PI )
		);
	joint_chain->Add(
		Twist( Vec3d( 0, 1, 0 ), Vec3d( 0, 0, 1.5 ) ),
		Link( Mat4d::Identity() ),
		Limits( -M_PI, M_PI )
		);

	return joint_chain;
}

// ------------------------------------------------------------
// Test fixture for HybridSolverAnalyzer
// ------------------------------------------------------------

class HybridSolverAnalyzerTest : public ::testing::Test
{
protected:
void SetUp() override
{
	// Create a test joint chain
	joint_chain_ = CreateTestJointChain();

	// Create a home configuration
	home_configuration_ = Mat4d::Identity();
}

void TearDown() override
{
}

std::shared_ptr< JointChain > joint_chain_;
Mat4d home_configuration_;
};

// ------------------------------------------------------------
// Test cases
// ------------------------------------------------------------

TEST_F( HybridSolverAnalyzerTest, AnalyzeConfiguration_WithBaseNumericWrist )
{
	// Analyze the configuration
	HybridSolverConfiguration config = HybridSolverAnalyzer::AnalyzeConfiguration( *joint_chain_, home_configuration_ );

	// Check that all components are detected
	EXPECT_TRUE( config.base_joint_model.has_value() ) << "Should detect base joint";
	EXPECT_TRUE( config.numeric_joints_model.has_value() ) << "Should detect numeric joints";
	EXPECT_TRUE( config.wrist_model.has_value() ) << "Should detect wrist";

	// Check that all flags are set
	EXPECT_EQ( config.solver_flags & HybridSolverFlags::Base, HybridSolverFlags::Base ) << "Base flag should be set";
	EXPECT_EQ( config.solver_flags & HybridSolverFlags::Numeric, HybridSolverFlags::Numeric ) << "Numeric flag should be set";
	EXPECT_EQ( config.solver_flags & HybridSolverFlags::Wrist, HybridSolverFlags::Wrist ) << "Wrist flag should be set";
}

// ------------------------------------------------------------

TEST_F( HybridSolverAnalyzerTest, AnalyzeConfiguration_WithBaseWristOnly )
{
	// Create a joint chain with only base and wrist joints (no numeric joints)
	auto simple_chain = std::make_shared< JointChain >( 3 );

	// Base joint
	Vec3d origin = Vec3d::Zero();
	simple_chain->Add(
		Twist( Vec3d( 0, 0, 1 ), Vec3d( 0, 0, 0 ) ),
		Link( Mat4d::Identity() ),
		Limits( -M_PI, M_PI )
		);

	// Wrist joints (2 joints)
	origin = Vec3d( 0, 0.5, 0 );
	simple_chain->Add(
		Twist( Vec3d( 1, 0, 0 ), origin ),
		Link( ToTransformMatrix( origin ) ),
		Limits( -M_PI, M_PI )
		);
	simple_chain->Add(
		Twist( Vec3d( 0, 1, 0 ), origin ),
		Link( ToTransformMatrix( origin ) ),
		Limits( -M_PI, M_PI )
		);

	// Analyze the configuration
	HybridSolverConfiguration config = HybridSolverAnalyzer::AnalyzeConfiguration( *simple_chain, home_configuration_ );

	// Check that base and wrist components are detected
	EXPECT_TRUE( config.base_joint_model.has_value() ) << "Should detect base joint";
	EXPECT_FALSE( config.numeric_joints_model.has_value() ) << "Should not detect numeric joints";
	EXPECT_TRUE( config.wrist_model.has_value() ) << "Should detect wrist";

	// Check that the correct flags are set
	EXPECT_EQ( config.solver_flags & HybridSolverFlags::Base, HybridSolverFlags::Base ) << "Base flag should be set";
	EXPECT_EQ( config.solver_flags & HybridSolverFlags::Numeric, HybridSolverFlags::None ) << "Numeric flag should not be set";
	EXPECT_EQ( config.solver_flags & HybridSolverFlags::Wrist, HybridSolverFlags::Wrist ) << "Wrist flag should be set";
}

// ------------------------------------------------------------

TEST_F( HybridSolverAnalyzerTest, AnalyzeConfiguration_WithWristOnly )
{
	// Create a joint chain with only a base joint
	auto base_chain = std::make_shared< JointChain >( 1 );

	// Base joint
	base_chain->Add(
		Twist( Vec3d( 0, 0, 1 ), Vec3d( 0, 0, 0 ) ),
		Link( Mat4d::Identity() ),
		Limits( -M_PI, M_PI )
		);

	// Analyze the configuration
	HybridSolverConfiguration config = HybridSolverAnalyzer::AnalyzeConfiguration( *base_chain, home_configuration_ );

	// Check that only base components are detected
	EXPECT_FALSE( config.base_joint_model.has_value() ) << "Should detect base joint";
	EXPECT_FALSE( config.numeric_joints_model.has_value() ) << "Should not detect numeric joints";
	EXPECT_TRUE( config.wrist_model.has_value() ) << "Should not detect wrist";

	// Check that the correct flags are set
	EXPECT_EQ( config.solver_flags & HybridSolverFlags::Base, HybridSolverFlags::None ) << "Base flag should be set";
	EXPECT_EQ( config.solver_flags & HybridSolverFlags::Numeric, HybridSolverFlags::None ) << "Numeric flag should not be set";
	EXPECT_EQ( config.solver_flags & HybridSolverFlags::Wrist, HybridSolverFlags::Wrist ) << "Wrist flag should not be set";
}

// ------------------------------------------------------------

TEST_F( HybridSolverAnalyzerTest, AnalyzeConfiguration_EmptyChain )
{
	// Create an empty joint chain
	auto empty_chain = std::make_shared< JointChain >( 0 );

	// Analyze the configuration
	HybridSolverConfiguration config = HybridSolverAnalyzer::AnalyzeConfiguration( *empty_chain, home_configuration_ );

	// Check that no components are detected
	EXPECT_FALSE( config.base_joint_model.has_value() ) << "Should not detect base joint";
	EXPECT_FALSE( config.numeric_joints_model.has_value() ) << "Should not detect numeric joints";
	EXPECT_FALSE( config.wrist_model.has_value() ) << "Should not detect wrist";

	// Check that no flags are set
	EXPECT_EQ( config.solver_flags, HybridSolverFlags::None ) << "No flags should be set";
}

// ------------------------------------------------------------

TEST_F( HybridSolverAnalyzerTest, AnalyzeConfiguration_WithNonIdentityHomeConfiguration )
{
	// Create a non-identity home configuration
	Mat4d non_identity_home = Mat4d::Identity();
	non_identity_home.block< 3, 3 >( 0, 0 ) = Eigen::AngleAxisd( M_PI / 4, Vec3d( 0, 0, 1 ) ).toRotationMatrix();
	non_identity_home.block< 3, 1 >( 0, 3 ) = Vec3d( 0.1, 0.2, 0.3 );

	// Analyze the configuration
	HybridSolverConfiguration config = HybridSolverAnalyzer::AnalyzeConfiguration( *joint_chain_, non_identity_home );

	// Check that all components are detected
	EXPECT_TRUE( config.base_joint_model.has_value() ) << "Should detect base joint";
	EXPECT_TRUE( config.numeric_joints_model.has_value() ) << "Should detect numeric joints";
	EXPECT_TRUE( config.wrist_model.has_value() ) << "Should detect wrist";

	// Check that all flags are set
	EXPECT_EQ( config.solver_flags & HybridSolverFlags::Base, HybridSolverFlags::Base ) << "Base flag should be set";
	EXPECT_EQ( config.solver_flags & HybridSolverFlags::Numeric, HybridSolverFlags::Numeric ) << "Numeric flag should be set";
	EXPECT_EQ( config.solver_flags & HybridSolverFlags::Wrist, HybridSolverFlags::Wrist ) << "Wrist flag should be set";
}

// ------------------------------------------------------------

} // namespace SOArm100::Kinematics::Test
