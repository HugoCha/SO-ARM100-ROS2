#pragma once

#include "KinematicsSolver.hpp"
#include "Types.hpp"

namespace SOArm100::Kinematics
{
class HybridKinematicsSolver : public KinematicsSolver
{
public:
HybridKinematicsSolver();
~HybridKinematicsSolver();

virtual bool InverseKinematic(
	const geometry_msgs::msg::Pose& target_pose,
	const std::span< const double >& seed_joints,
	std::vector< double >& joints ) const override;

private:
Mat4d ComputeWristCenterPose( const geometry_msgs::msg::Pose& target_pose );
Mat4d ComputeShoulderPose( double base_joint );
double ComputeBaseJoint( const Mat4d& wrist_pose );
std::vector< double > ComputeIntermediateJoints( const Mat4d& wrist_pose );
std::vector< double > ComputeWristJoints(
	const Mat4d& base_to_wrist_center,
	const Mat4d& target_pose );
};
}
