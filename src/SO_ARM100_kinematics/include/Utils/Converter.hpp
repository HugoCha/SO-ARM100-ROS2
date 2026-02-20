#pragma once

#include "Global.hpp"

#include <geometry_msgs/geometry_msgs/msg/pose.hpp>
#include <span>
#include <string>
#include <vector>

namespace SOArm100::Kinematics
{
[[nodiscard]] VecXd ToVecXd( const std::span< const double >& vec );
[[nodiscard]] std::vector< double > ToStdVector( const VecXd& vec );

[[nodiscard]] geometry_msgs::msg::Pose ToPoseMsg( const Mat4d& matrix );
[[nodiscard]] geometry_msgs::msg::Pose ToPoseMsg( const Iso3d& matrix );

[[nodiscard]] Mat4d ToTransformMatrix( const geometry_msgs::msg::Pose& pose_msg );
[[nodiscard]] Mat4d ToTransformMatrix( const Mat3d& rotation, const Vec3d translation );
[[nodiscard]] Mat4d ToTransformMatrix( const Mat3d& rotation );
[[nodiscard]] Mat4d ToTransformMatrix( const Vec3d& translation );

[[nodiscard]] const std::string ToString( const geometry_msgs::msg::Pose& pose );
}
