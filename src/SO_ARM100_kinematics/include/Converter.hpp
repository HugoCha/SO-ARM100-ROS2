#pragma once

#include "Types.hpp"

#include <geometry_msgs/geometry_msgs/msg/pose.hpp>
#include <vector>

namespace SOArm100::Kinematics
{
VecXd ToVecXd( const std::vector< double >& vec );
std::vector< double > ToStdVector( const VecXd& vec );

Mat4d ToMat4d( const geometry_msgs::msg::Pose& pose_msg );
geometry_msgs::msg::Pose ToPoseMsg( const Mat4d& matrix );
}
