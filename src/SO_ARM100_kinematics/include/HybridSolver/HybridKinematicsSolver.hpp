#pragma once

#include "Global.hpp"

#include "IKinematicsSolver.hpp"
#include "KinematicsSolver.hpp"

namespace SOArm100::Kinematics
{
class HybridKinematicsSolver : public KinematicsSolver
{
public:
HybridKinematicsSolver();
~HybridKinematicsSolver();

virtual void Initialize(
	const moveit::core::RobotModelConstPtr& robot_model,
	const std::string& group_name,
	const std::string& base_frame,
	const std::vector< std::string >& tip_frames,
	double search_discretization ) override;

protected:
virtual bool InverseKinematic(
	const Mat4d& target_pose,
	const std::span< const double >& seed_joints,
	VecXd& joints ) const override;

private:
std::unique_ptr< IKinematicsSolver > solver_;
double search_discretization_;
};
}
