#include "Joint/JointChain.hpp"

#include "RobotModelTestData.hpp"

#include <gtest/gtest.h>
#include <memory>

namespace SOArm100::Kinematics::Test
{

class JointChainTest : public ::testing::Test
{
protected:
void SetUp() override {
}
void TearDown() override {
}
};

// ------------------------------------------------------------

TEST_F( JointChainTest, ConstructorWithCapacity )
{
	int n = 10;
	JointChain chain( n );
	ASSERT_EQ( chain.GetJointCount(), 0 );
	ASSERT_EQ( chain.GetActiveJointCount(), 0 );
	ASSERT_TRUE( chain.Empty() );
}

// ------------------------------------------------------------

TEST_F( JointChainTest, GetRevoluteOnlyRobotJointChain )
{
	JointChain chain = Data::GetRevoluteOnlyRobotJointChain();

	ASSERT_EQ( chain.GetJointCount(), 3 );
	ASSERT_EQ( chain.GetActiveJointCount(), 3 );

	// Check each joint's properties
	for ( int i = 0; i < 3; ++i )
	{
		const Twist& twist = chain.GetActiveJointTwist( i );
		const Link& link = chain.GetActiveJointLink( i );
		const Limits& limits = chain.GetActiveJointLimits( i );

		ASSERT_EQ( limits.Min(), -M_PI );
		ASSERT_EQ( limits.Max(), M_PI );
	}

	// Check specific joint properties
	const Twist& twist1 = chain.GetActiveJointTwist( 0 );
	ASSERT_TRUE( twist1.GetAxis().isApprox( Vec3d( 0, 0, 1 ) ) );
	ASSERT_TRUE( twist1.GetLinear().isApprox( Vec3d( 0, 0, 0 ) ) );

	const Twist& twist2 = chain.GetActiveJointTwist( 1 );
	ASSERT_TRUE( twist2.GetAxis().isApprox( Vec3d( 0, 1, 0 ) ) );
	ASSERT_TRUE( twist2.GetLinear().isApprox( Vec3d( 0, 0, 0.5 ) ) );

	const Twist& twist3 = chain.GetActiveJointTwist( 2 );
	ASSERT_TRUE( twist3.GetAxis().isApprox( Vec3d( 0, 0, 1 ) ) );
	ASSERT_TRUE( twist3.GetLinear().isApprox( Vec3d( 0, -1, 0 ) ) ) << twist3.GetLinear();
}

// ------------------------------------------------------------

TEST_F( JointChainTest, SubChain )
{
	JointChain chain = Data::GetRevoluteOnlyRobotJointChain();

	// Get the first and last joints
	JointConstPtr start = chain.GetActiveJoint( 0 );
	JointConstPtr end = chain.GetActiveJoint( 2 );

	// Create a subchain from the first to the last joint
	JointChain subchain = chain.SubChain( start, end );

	ASSERT_EQ( subchain.GetJointCount(), 3 );
	ASSERT_EQ( subchain.GetActiveJointCount(), 3 );

	// Check that the subchain has the same joints as the original chain
	for ( int i = 0; i < 3; ++i )
	{
		const Twist& original_twist = chain.GetActiveJointTwist( i );
		const Twist& subchain_twist = subchain.GetActiveJointTwist( i );
		ASSERT_TRUE( original_twist.GetAxis().isApprox( subchain_twist.GetAxis() ) );
		ASSERT_TRUE( original_twist.GetLinear().isApprox( subchain_twist.GetLinear() ) );
	}
}

// ------------------------------------------------------------

TEST_F( JointChainTest, SubChainInvalidInputs )
{
	JointChain chain = Data::GetRevoluteOnlyRobotJointChain();

	// Test with null pointers
	ASSERT_THROW( auto sub_chain = chain.SubChain( nullptr, chain.GetActiveJoint( 0 ) ), std::out_of_range );
	ASSERT_THROW( auto sub_chain = chain.SubChain( chain.GetActiveJoint( 0 ), nullptr ), std::out_of_range );

	// Test with invalid indices (joints not in the chain)
	JointConstPtr joint_not_in_chain = std::make_shared< const Joint >(
		Twist( Vec3d( 1, 0, 0 ), Vec3d( 0, 0, 0 ) ),
		Link(),
		Limits( -M_PI, M_PI )
		);
	ASSERT_THROW( auto sub_chain = chain.SubChain( joint_not_in_chain, chain.GetActiveJoint( 0 ) ), std::out_of_range );
	ASSERT_THROW( auto sub_chain = chain.SubChain( chain.GetActiveJoint( 0 ), joint_not_in_chain ), std::out_of_range );

	// Test with start index > end index
	ASSERT_THROW( auto sub_chain = chain.SubChain( chain.GetActiveJoint( 2 ), chain.GetActiveJoint( 0 ) ), std::out_of_range );
}

// ------------------------------------------------------------

TEST_F( JointChainTest, AddFunction )
{
	JointChain chain( 1 );

	// Add a joint
	Twist twist( Vec3d( 0, 0, 1 ), Vec3d( 0, 0, 0 ) );
	Link link;
	Limits limits( -M_PI, M_PI );

	chain.Add( twist, link, limits );

	ASSERT_EQ( chain.GetJointCount(), 1 );
	ASSERT_EQ( chain.GetActiveJointCount(), 1 );

	// Check the added joint's properties
	const Twist& added_twist = chain.GetActiveJointTwist( 0 );
	ASSERT_TRUE( added_twist.GetAxis().isApprox( Vec3d( 0, 0, 1 ) ) );
	ASSERT_TRUE( added_twist.GetLinear().isApprox( Vec3d( 0, 0, 0 ) ) );

	const Limits& added_limits = chain.GetActiveJointLimits( 0 );
	ASSERT_EQ( added_limits.Min(), -M_PI );
	ASSERT_EQ( added_limits.Max(), M_PI );
}

// ------------------------------------------------------------

TEST_F( JointChainTest, GetActiveJointInvalidIndex )
{
	JointChain chain = Data::GetRevoluteOnlyRobotJointChain();
	ASSERT_THROW( chain.GetActiveJoint( 3 ), std::out_of_range );
	ASSERT_THROW( chain.GetActiveJointTwist( 3 ), std::out_of_range );
	ASSERT_THROW( chain.GetActiveJointLimits( 3 ), std::out_of_range );
	ASSERT_THROW( chain.GetActiveJointLink( 3 ), std::out_of_range );
}

// ------------------------------------------------------------

} // namespace SOArm100::Kinematics::Test
