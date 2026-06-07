#pragma once

#include <rclcpp/rclcpp.hpp>
#include <memory>

namespace SOArm100::Kinematics
{

class Logger
{
public:
static void init( const rclcpp::Logger& logger )
{
	logger_ = std::make_unique< rclcpp::Logger >( logger );
}

static rclcpp::Logger& get()
{
	if ( !logger_ )
	{
		Logger::init( rclcpp::get_logger( "RAKL" ) );
	}
	return *logger_;
}

private:
inline static std::unique_ptr< rclcpp::Logger > logger_ = nullptr;
};

}