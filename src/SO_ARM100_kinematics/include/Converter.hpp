#pragma once

#include "Types.hpp"

#include <geometry_msgs/geometry_msgs/msg/pose.hpp>
#include <span>
#include <string>
#include <vector>

namespace SOArm100::Kinematics
{
[[nodiscard]] VecXd ToVecXd( const std::span< const double >& vec );
[[nodiscard]] std::vector< double > ToStdVector( const VecXd& vec );

[[nodiscard]] Mat4d ToMat4d( const geometry_msgs::msg::Pose& pose_msg );
[[nodiscard]] geometry_msgs::msg::Pose ToPoseMsg( const Mat4d& matrix );

[[nodiscard]] std::string ToString( const geometry_msgs::msg::Pose& pose );
}
