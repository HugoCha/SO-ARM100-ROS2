#include "Model/TotalLengthReachableSpace.hpp"

#include "Global.hpp"
#include "RobotModelTestData.hpp"

#include "Model/JointChain.hpp"

#include <gtest/gtest.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace SOArm100::Kinematics::Test
{

// ------------------------------------------------------------
// ------------------------------------------------------------

class TotalLengthReachableSpaceTest : public ::testing::Test
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
	reachable_space_.reset( new Model::TotalLengthReachableSpace( joint_chain, home ) );
}

void TearDown() override
{
	// Clean up resources if needed
}

protected:
std::unique_ptr< Model::TotalLengthReachableSpace > reachable_space_;
};

// ------------------------------------------------------------
// ------------------------------------------------------------

TEST_F( TotalLengthReachableSpaceTest, IsUnreachable )
{
	Mat4d target_pose;

	// Set a target pose that is unreachable
	target_pose.x() = 100.0;
	target_pose.y() = 100.0;
	target_pose.z() = 100.0;

	bool is_unreachable = reachable_space_->IsUnreachable( target_pose );
	EXPECT_TRUE( is_unreachable ) << "Target pose should be unreachable";
}

// ------------------------------------------------------------

TEST_F( TotalLengthReachableSpaceTest, IsReachable )
{
	Mat4d target_pose;

	// Set a target pose that is reachable
	target_pose.x() = 0.5;
	target_pose.y() = 0.0;
	target_pose.z() = 0.0;

	bool is_unreachable = reachable_space_->IsUnreachable( target_pose );
	EXPECT_FALSE( is_unreachable ) << "Target pose should be reachable";
}

// ------------------------------------------------------------

} // namespace SOArm100::Kinematics::Test
