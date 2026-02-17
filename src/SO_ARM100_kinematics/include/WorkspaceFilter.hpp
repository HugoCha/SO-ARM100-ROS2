#pragma once

#include "Global.hpp"

#include <geometry_msgs/msg/pose.hpp>

namespace SOArm100::Kinematics
{
class JointChain;

class WorkspaceFilter
{
public:
WorkspaceFilter( const JointChain& joint_chain );

[[nodiscard]] bool IsUnreachable( const geometry_msgs::msg::Pose& target_pose ) const;
[[nodiscard]] bool IsUnreachable( const Mat4d& target_pose ) const;

protected:
Vec3d base_frame_;

virtual void ComputeWorkspace( const JointChain& joint_chain );

[[nodiscard]] virtual double ComputeMaxReach( const JointChain& joint_chain ) const;
[[nodiscard]] virtual double ComputeMinReach( const JointChain& joint_chain ) const;

[[nodiscard]] const Vec3d GetBaseFrame() const;

private:
double min_reach_;
double max_reach_;
};
}