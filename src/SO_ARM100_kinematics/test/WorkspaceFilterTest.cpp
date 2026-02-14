#include "WorkspaceFilter.hpp"

#include "JointChain.hpp"
#include "RobotModelTestData.hpp"

#include <gtest/gtest.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace SOArm100::Kinematics::Test
{

// ------------------------------------------------------------
// ------------------------------------------------------------

class WorkspaceFilterTest : public ::testing::Test
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
	const auto& joint_chain = Data::GetRevoluteOnlyRobotJointChain();     // Replace with your robot model creation logic
	workspace_filter_.reset( new WorkspaceFilter( joint_chain ) );
}

void TearDown() override
{
	// Clean up resources if needed
}

protected:
std::unique_ptr< WorkspaceFilter > workspace_filter_;
};

// ------------------------------------------------------------
// ------------------------------------------------------------

TEST_F( WorkspaceFilterTest, IsUnreachable )
{
	geometry_msgs::msg::Pose target_pose;

	// Set a target pose that is unreachable
	target_pose.position.x = 100.0;
	target_pose.position.y = 100.0;
	target_pose.position.z = 100.0;

	bool is_unreachable = workspace_filter_->IsUnreachable( target_pose );
	EXPECT_TRUE( is_unreachable ) << "Target pose should be unreachable";
}

// ------------------------------------------------------------

TEST_F( WorkspaceFilterTest, IsReachable )
{
	geometry_msgs::msg::Pose target_pose;

	// Set a target pose that is reachable
	target_pose.position.x = 0.5;
	target_pose.position.y = 0.0;
	target_pose.position.z = 0.0;

	bool is_unreachable = workspace_filter_->IsUnreachable( target_pose );
	EXPECT_FALSE( is_unreachable ) << "Target pose should be reachable";
}

// ------------------------------------------------------------

} // namespace SOArm100::Kinematics::Test
