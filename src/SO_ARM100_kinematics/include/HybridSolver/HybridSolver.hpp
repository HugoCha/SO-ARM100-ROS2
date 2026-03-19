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

virtual void Initialize(
	std::shared_ptr< const JointChain > joint_chain,
	std::shared_ptr< const Mat4d > home_configuration,
	double search_discretization ) override;

protected:
virtual bool InverseKinematicImpl(
	const Mat4d& target,
	const std::span< const double >& seed_joints,
	double* joints ) const override;

private:
std::unique_ptr< IKinematicsSolver > solver_;
double search_discretization_;
};
}
