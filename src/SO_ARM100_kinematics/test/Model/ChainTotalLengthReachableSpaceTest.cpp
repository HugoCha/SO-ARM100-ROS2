#include "Model/ReachableSpace/ChainTotalLengthReachableSpace.hpp"

#include "Global.hpp"
#include "RobotModelTestData.hpp"

#include "Model/Joint/JointChain.hpp"

#include <gtest/gtest.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace SOArm100::Kinematics::Test
{

// ------------------------------------------------------------
// ------------------------------------------------------------

class ChainTotalLengthReachableSpaceTest : public ::testing::Test
{
protected:
void SetUp() override
{
	// Initialize ROS context for logger support
	if ( !rclcpp::ok() )
	{
		int argc = 0;
		char** argv = nullptr;
		rclcpp::init( argc, argv );
	}

	// Create a robot model for testing
	Model::JointChain joint_chain = Data::GetZYZRevoluteRobotJointChain();
	Mat4d home = Data::GetZYZRevoluteRobotHome();
	reachable_space_.reset( new Model::ChainTotalLengthReachableSpace( joint_chain, home ) );
}

void TearDown() override
{
	// Clean up resources if needed
}

protected:
std::unique_ptr< Model::ChainTotalLengthReachableSpace > reachable_space_;
};

// ------------------------------------------------------------
// ------------------------------------------------------------

TEST_F( ChainTotalLengthReachableSpaceTest, IsUnreachable )
{
	Mat4d target_pose;

	// Set a target pose that is unreachable
	target_pose( 0, 3 ) = 100.0;
	target_pose( 1, 3 ) = 100.0;
	target_pose( 2, 3 ) = 100.0;

	bool is_unreachable = reachable_space_->IsUnreachable( target_pose );
	EXPECT_TRUE( is_unreachable ) << "Target pose should be unreachable";
}

// ------------------------------------------------------------

TEST_F( ChainTotalLengthReachableSpaceTest, IsReachable )
{
	Mat4d target_pose;

	// Set a target pose that is reachable
	target_pose( 0, 3 ) = 0.5;
	target_pose( 1, 3 ) = 0.0;
	target_pose( 2, 3 ) = 0.0;

	bool is_unreachable = reachable_space_->IsUnreachable( target_pose );
	EXPECT_FALSE( is_unreachable ) << "Target pose should be reachable";
}

// ------------------------------------------------------------

} // namespace SOArm100::Kinematics::Test
