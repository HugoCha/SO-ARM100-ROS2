#include "Model/Joint/JointChain.hpp"

#include "Global.hpp"
#include "Model/Joint/Joint.hpp"
#include "Model/Joint/JointState.hpp"
#include "Model/Pose.hpp"
#include "RobotModelTestData.hpp"
#include "Utils/Converter.hpp"
#include "Utils/KinematicsUtils.hpp"

#include <Eigen/src/Geometry/AngleAxis.h>
#include <gtest/gtest.h>
#include <memory>

namespace SOArm100::Kinematics::Test
{

class JointChainTest : public ::testing::Test
{
protected:
void SetUp() override {
	// Create a joint chain with 3 joints
	joint_chain_ = std::make_shared< Model::JointChain >( 3 );

	// Create joints
	joint_chain_->Add(
		Model::Twist( Vec3d( 0, 0, 1 ), Vec3d( 0, 0, 0 ) ),
		Model::Link( Mat4d::Identity(), 0.5 ),
		Model::Limits( -M_PI, M_PI )
		);

	joint_chain_->Add(
		Model::Twist( Vec3d( 0, 1, 0 ), Vec3d( 0, 0, 0.5 ) ),
		Model::Link( ToTransformMatrix( Vec3d( 0, 0, 0.5 ) ), 0.5 ),
		Model::Limits( -M_PI / 2, M_PI / 2 )
		);

	joint_chain_->Add(
		Model::Twist( Vec3d( 1, 0, 0 ), Vec3d( 0, 0, 1.0 ) ),
		Model::Link( ToTransformMatrix( Vec3d( 0, 0, 1.0 ) ), 0 ),
		Model::Limits( -M_PI, M_PI )
		);
	joint1_ = joint_chain_->GetJoints()[0];
	joint2_ = joint_chain_->GetJoints()[1];
	joint3_ = joint_chain_->GetJoints()[2];
}
void TearDown() override {
}

std::shared_ptr< Model::JointChain > joint_chain_;
std::shared_ptr< const Model::Joint > joint1_;
std::shared_ptr< const Model::Joint > joint2_;
std::shared_ptr< const Model::Joint > joint3_;
};

// ------------------------------------------------------------

TEST_F( JointChainTest, ConstructorWithCapacity )
{
	int n = 10;
	Model::JointChain chain( n );
	ASSERT_EQ( chain.GetJointCount(), 0 );
	ASSERT_EQ( chain.GetActiveJointCount(), 0 );
	ASSERT_TRUE( chain.Empty() );
}

// ------------------------------------------------------------

TEST_F( JointChainTest, GetZYZRevoluteRobotJointChain )
{
	Model::JointChain chain = Data::GetZYZRevoluteRobotJointChain();

	ASSERT_EQ( chain.GetJointCount(), 3 );
	ASSERT_EQ( chain.GetActiveJointCount(), 3 );

	// Check each joint's properties
	for ( int i = 0; i < 3; ++i )
	{
		const Model::Twist& twist = chain.GetActiveJointTwist( i );
		const Model::Link& link = chain.GetActiveJointLink( i );
		const Model::Limits& limits = chain.GetActiveJointLimits( i );

		ASSERT_EQ( limits.Min(), -M_PI );
		ASSERT_EQ( limits.Max(), M_PI );
	}

	// Check specific joint properties
	const Model::Twist& twist1 = chain.GetActiveJointTwist( 0 );
	ASSERT_TRUE( twist1.Omega().isApprox( Vec3d( 0, 0, 1 ) ) );
	ASSERT_TRUE( twist1.V().isApprox( Vec3d( 0, 0, 0 ) ) );

	const Model::Twist& twist2 = chain.GetActiveJointTwist( 1 );
	ASSERT_TRUE( twist2.Omega().isApprox( Vec3d( 0, 1, 0 ) ) );
	ASSERT_TRUE( twist2.V().isApprox( Vec3d( 0, 0, 0.5 ) ) );

	const Model::Twist& twist3 = chain.GetActiveJointTwist( 2 );
	ASSERT_TRUE( twist3.Omega().isApprox( Vec3d( 0, 0, 1 ) ) );
	ASSERT_TRUE( twist3.V().isApprox( Vec3d( 0, -1, 0 ) ) ) << twist3.V();
}

// ------------------------------------------------------------

TEST_F( JointChainTest, SubChain )
{
	Model::JointChain chain = Data::GetZYZRevoluteRobotJointChain();

	// Get the first and last joints
	Model::JointConstPtr start = chain.GetActiveJoint( 0 );
	Model::JointConstPtr end = chain.GetActiveJoint( 2 );

	// Create a subchain from the first to the last joint
	Model::JointChain subchain = chain.SubChain( start, end );

	ASSERT_EQ( subchain.GetJointCount(), 3 );
	ASSERT_EQ( subchain.GetActiveJointCount(), 3 );

	// Check that the subchain has the same joints as the original chain
	for ( int i = 0; i < 3; ++i )
	{
		const Model::Twist& original_twist = chain.GetActiveJointTwist( i );
		const Model::Twist& subchain_twist = subchain.GetActiveJointTwist( i );
		ASSERT_TRUE( original_twist.Omega().isApprox( subchain_twist.Omega() ) );
		ASSERT_TRUE( original_twist.V().isApprox( subchain_twist.V() ) );
	}
}

// ------------------------------------------------------------

TEST_F( JointChainTest, SubChainInvalidInputs )
{
	Model::JointChain chain = Data::GetZYZRevoluteRobotJointChain();

	// Test with null pointers
	ASSERT_THROW( auto sub_chain = chain.SubChain( nullptr, chain.GetActiveJoint( 0 ) ), std::out_of_range );
	ASSERT_THROW( auto sub_chain = chain.SubChain( chain.GetActiveJoint( 0 ), nullptr ), std::out_of_range );

	// Test with invalid indices (joints not in the chain)
	Model::JointConstPtr joint_not_in_chain = std::make_shared< const Model::Joint >(
		Model::Twist( Vec3d( 1, 0, 0 ), Vec3d( 0, 0, 0 ) ),
		Model::Link(),
		Model::Limits( -M_PI, M_PI )
		);
	ASSERT_THROW( auto sub_chain = chain.SubChain( joint_not_in_chain, chain.GetActiveJoint( 0 ) ), std::out_of_range );
	ASSERT_THROW( auto sub_chain = chain.SubChain( chain.GetActiveJoint( 0 ), joint_not_in_chain ), std::out_of_range );

	// Test with start index > end index
	ASSERT_THROW( auto sub_chain = chain.SubChain( chain.GetActiveJoint( 2 ), chain.GetActiveJoint( 0 ) ), std::out_of_range );
}

// ------------------------------------------------------------

TEST_F( JointChainTest, AddFunction )
{
	Model::JointChain chain( 1 );

	// Add a joint
	Model::Twist twist( Vec3d( 0, 0, 1 ), Vec3d( 0, 0, 0 ) );
	Model::Link link;
	Model::Limits limits( -M_PI, M_PI );

	chain.Add( twist, link, limits );

	ASSERT_EQ( chain.GetJointCount(), 1 );
	ASSERT_EQ( chain.GetActiveJointCount(), 1 );

	// Check the added joint's properties
	const Model::Twist& added_twist = chain.GetActiveJointTwist( 0 );
	ASSERT_TRUE( added_twist.Omega().isApprox( Vec3d( 0, 0, 1 ) ) );
	ASSERT_TRUE( added_twist.V().isApprox( Vec3d( 0, 0, 0 ) ) );

	const Model::Limits& added_limits = chain.GetActiveJointLimits( 0 );
	ASSERT_EQ( added_limits.Min(), -M_PI );
	ASSERT_EQ( added_limits.Max(), M_PI );
}

// ------------------------------------------------------------

TEST_F( JointChainTest, GetActiveJointInvalidIndex )
{
	Model::JointChain chain = Data::GetZYZRevoluteRobotJointChain();
	ASSERT_THROW( chain.GetActiveJoint( 3 ), std::out_of_range );
	ASSERT_THROW( chain.GetActiveJointTwist( 3 ), std::out_of_range );
	ASSERT_THROW( chain.GetActiveJointLimits( 3 ), std::out_of_range );
	ASSERT_THROW( chain.GetActiveJointLink( 3 ), std::out_of_range );
}

// ------------------------------------------------------------

TEST_F( JointChainTest, GetJointIndex_ValidJoints )
{
	// Test with valid joints
	EXPECT_EQ( joint_chain_->GetJointIndex( joint1_.get() ), 0 ) << "Should return index 0 for joint1";
	EXPECT_EQ( joint_chain_->GetJointIndex( joint2_.get() ), 1 ) << "Should return index 1 for joint2";
	EXPECT_EQ( joint_chain_->GetJointIndex( joint3_.get() ), 2 ) << "Should return index 2 for joint3";
}

// ------------------------------------------------------------

TEST_F( JointChainTest, GetJointIndex_NullJoint )
{
	// Test with null joint
	EXPECT_EQ( joint_chain_->GetJointIndex( nullptr ), -1 ) << "Should return -1 for null joint";
}

// ------------------------------------------------------------

TEST_F( JointChainTest, GetJointIndex_EmptyChain )
{
	// Create an empty joint chain
	Model::JointChain empty_chain( 0 );

	// Test with empty chain
	EXPECT_EQ( empty_chain.GetJointIndex( joint1_.get() ), -1 ) << "Should return -1 for empty chain";
}

// ------------------------------------------------------------

TEST_F( JointChainTest, GetJointIndex_NonExistentJoint )
{
	// Create a joint not in the chain
	auto other_joint = std::make_shared< Model::Joint >(
		Model::Twist( Vec3d( 0, 0, 1 ), Vec3d( 0, 0, 0 ) ),
		Model::Link( Mat4d::Identity(), 0 ),
		Model::Limits( -M_PI, M_PI )
		);

	// Test with non-existent joint
	EXPECT_EQ( joint_chain_->GetJointIndex( other_joint.get() ), -1 ) << "Should return -1 for non-existent joint";
}

// ------------------------------------------------------------

TEST_F( JointChainTest, GetNextJoint_ValidJoints )
{
	// Test with valid joints
	EXPECT_EQ( joint_chain_->GetNextJoint( joint1_.get() ), joint2_.get() ) << "Should return joint2 for joint1";
	EXPECT_EQ( joint_chain_->GetNextJoint( joint2_.get() ), joint3_.get() ) << "Should return joint3 for joint2";
	EXPECT_EQ( joint_chain_->GetNextJoint( joint3_.get() ), nullptr ) << "Should return nullptr for last joint";
}

// ------------------------------------------------------------

TEST_F( JointChainTest, GetNextJoint_NullJoint )
{
	// Test with null joint
	EXPECT_EQ( joint_chain_->GetNextJoint( nullptr ), nullptr ) << "Should return nullptr for null joint";
}

// ------------------------------------------------------------

TEST_F( JointChainTest, GetNextJoint_NonExistentJoint )
{
	// Create a joint not in the chain
	auto other_joint = std::make_shared< Model::Joint >(
		Model::Twist( Vec3d( 0, 0, 1 ), Vec3d( 0, 0, 0 ) ),
		Model::Link( Mat4d::Identity(), 0 ),
		Model::Limits( -M_PI, M_PI )
		);

	// Test with non-existent joint
	EXPECT_EQ( joint_chain_->GetNextJoint( other_joint.get() ), nullptr ) << "Should return nullptr for non-existent joint";
}

// ------------------------------------------------------------

TEST_F( JointChainTest, GetNextJoint_EmptyChain )
{
	// Create an empty joint chain
	Model::JointChain empty_chain( 0 );

	// Test with empty chain
	EXPECT_EQ( empty_chain.GetNextJoint( joint1_.get() ), nullptr ) << "Should return nullptr for empty chain";
}

// ------------------------------------------------------------

TEST_F( JointChainTest, GetPreviousJoint_ValidJoints )
{
	// Test with valid joints
	EXPECT_EQ( joint_chain_->GetPreviousJoint( joint1_.get() ), nullptr ) << "Should return nullptr for first joint";
	EXPECT_EQ( joint_chain_->GetPreviousJoint( joint2_.get() ), joint1_.get() ) << "Should return joint1 for joint2";
	EXPECT_EQ( joint_chain_->GetPreviousJoint( joint3_.get() ), joint2_.get() ) << "Should return joint2 for joint3";
}

// ------------------------------------------------------------

TEST_F( JointChainTest, GetPreviousJoint_NullJoint )
{
	// Test with null joint
	EXPECT_EQ( joint_chain_->GetPreviousJoint( nullptr ), nullptr ) << "Should return nullptr for null joint";
}

// ------------------------------------------------------------

TEST_F( JointChainTest, GetPreviousJoint_NonExistentJoint )
{
	// Create a joint not in the chain
	auto other_joint = std::make_shared< Model::Joint >(
		Model::Twist( Vec3d( 0, 0, 1 ), Vec3d( 0, 0, 0 ) ),
		Model::Link( Mat4d::Identity(), 0 ),
		Model::Limits( -M_PI, M_PI )
		);

	// Test with non-existent joint
	EXPECT_EQ( joint_chain_->GetPreviousJoint( other_joint.get() ), nullptr ) << "Should return nullptr for non-existent joint";
}

// ------------------------------------------------------------

TEST_F( JointChainTest, GetPreviousJoint_EmptyChain )
{
	// Create an empty joint chain
	Model::JointChain empty_chain( 0 );

	// Test with empty chain
	EXPECT_EQ( empty_chain.GetPreviousJoint( joint1_.get() ), nullptr ) << "Should return nullptr for empty chain";
}

// ------------------------------------------------------------

TEST_F( JointChainTest, Integration_GetJointIndexAndNextJoint )
{
	// Get index of joint2
	int index = joint_chain_->GetJointIndex( joint2_.get() );
	EXPECT_EQ( index, 1 ) << "Should return index 1 for joint2";

	// Get next joint of joint2
	const Model::Joint* next_joint = joint_chain_->GetNextJoint( joint2_.get() );
	EXPECT_EQ( next_joint, joint3_.get() ) << "Should return joint3 for joint2";
}

// ------------------------------------------------------------

TEST_F( JointChainTest, Integration_GetJointIndexAndPreviousJoint )
{
	// Get index of joint3
	int index = joint_chain_->GetJointIndex( joint3_.get() );
	EXPECT_EQ( index, 2 ) << "Should return index 2 for joint3";

	// Get previous joint of joint3
	const Model::Joint* prev_joint = joint_chain_->GetPreviousJoint( joint3_.get() );
	EXPECT_EQ( prev_joint, joint2_.get() ) << "Should return joint2 for joint3";
}

// ------------------------------------------------------------

TEST_F( JointChainTest, EdgeCases_AllFunctions )
{
	// Test with first joint
	EXPECT_EQ( joint_chain_->GetJointIndex( joint1_.get() ), 0 ) << "Should return index 0 for first joint";
	EXPECT_EQ( joint_chain_->GetPreviousJoint( joint1_.get() ), nullptr ) << "Should return nullptr for previous of first joint";
	EXPECT_NE( joint_chain_->GetNextJoint( joint1_.get() ), nullptr ) << "Should return non-null for next of first joint";

	// Test with last joint
	EXPECT_EQ( joint_chain_->GetJointIndex( joint3_.get() ), 2 ) << "Should return index 2 for last joint";
	EXPECT_NE( joint_chain_->GetPreviousJoint( joint3_.get() ), nullptr ) << "Should return non-null for previous of last joint";
	EXPECT_EQ( joint_chain_->GetNextJoint( joint3_.get() ), nullptr ) << "Should return nullptr for next of last joint";
}

// ------------------------------------------------------------

TEST_F( JointChainTest, RobotRevoluteOnly_ComputeFK )
{
	auto joint_chain = Data::GetZYZRevoluteRobotJointChain();
	auto home = Data::GetZYZRevoluteRobotHome();

	VecXd joints{ 3 };
	joints << M_PI / 8, M_PI / 2, 0.7;

	Mat4d expected = Data::GetZYZRevoluteRobotTransform( joints[0], joints[1], joints[2] );

	Mat4d result;
	joint_chain.ComputeFK( joints, home, result );

	EXPECT_TRUE( IsApprox( expected, result, 1e-9 ) );
}

// ------------------------------------------------------------

TEST_F( JointChainTest, RobotRevoluteOnly_ComputeJointPoses )
{
	auto joint_chain = Data::GetZYZRevoluteRobotJointChain();
	auto home = Data::GetZYZRevoluteRobotHome();

	VecXd joints{ 3 };
	joints << M_PI / 8, M_PI / 2, 0.7;

	Mat4d T_cumul_0 = Data::GetZYZRevoluteRobotT01( joints[0] );
	Mat4d T_cumul_1 = T_cumul_0 * Data::GetZYZRevoluteRobotT12( joints[1] );
	Mat4d T_cumul_2 = T_cumul_1 * Data::GetZYZRevoluteRobotT23( joints[2] );

	// World-frame axis = R(T_cumul_i) * local_axis
	std::vector< Model::Pose > expected_int_fk
	{
		Model::Pose{
			Translation( T_cumul_0 ),
			Rotation( T_cumul_0 ) * joint_chain.GetActiveJoint( 0 )->Axis()
		},
		Model::Pose{
			Translation( T_cumul_1 ),
			Rotation( T_cumul_1 ) * joint_chain.GetActiveJoint( 1 )->Axis()
		},
		Model::Pose{
			Translation( T_cumul_2 ),
			Rotation( T_cumul_2 ) * joint_chain.GetActiveJoint( 2 )->Axis()
		},
	};

	Mat4d expected_fk = Data::GetZYZRevoluteRobotTransform( joints[0], joints[1], joints[2] );

	std::vector< Model::JointState > result_int_fk;
	Mat4d result;
	joint_chain.ComputeJointStatesFK( joints, home, result_int_fk, result );

	for ( int i = 0; i < 3; i++ )
	{
		EXPECT_TRUE( ( expected_int_fk[i].origin - result_int_fk[i].Origin() ).norm() < 1e-9 )
		    << "Expected Origin " << i << " = " << std::endl << expected_int_fk[i].origin.transpose() << std::endl
		    << "Result Origin   " << i << " = " << std::endl << result_int_fk[i].Origin().transpose() << std::endl;
		EXPECT_TRUE( ( expected_int_fk[i].axis - result_int_fk[i].Axis() ).norm() < 1e-9 )
		    << "Expected Axis " << i << " = " << std::endl << expected_int_fk[i].axis.transpose() << std::endl
		    << "Result Axis   " << i << " = " << std::endl << result_int_fk[i].Axis().transpose() << std::endl;;
	}
	EXPECT_TRUE( IsApprox( expected_fk, result, 1e-9 ) )
	    << "Expected fk = " << std::endl << expected_fk << std::endl
	    << "Result fk   = " << std::endl << result << std::endl;
}

// ------------------------------------------------------------

} // namespace SOArm100::Kinematics::Test
