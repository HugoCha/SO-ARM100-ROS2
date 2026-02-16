#pragma once

#include "Global.hpp"

#include "Joint/JointChain.hpp"
#include "WorkspaceFilter.hpp"

#include <memory>
#include <moveit/macros/class_forward.hpp>
#include <span>

namespace moveit::core
{
MOVEIT_CLASS_FORWARD( JointModel );
MOVEIT_CLASS_FORWARD( JointModelGroup );
MOVEIT_CLASS_FORWARD( RobotModel );
}

namespace SOArm100::Kinematics
{

class KinematicsSolver
{
public:
KinematicsSolver();
~KinematicsSolver() = default;

virtual void Initialize(
	const moveit::core::RobotModelConstPtr& robot_model,
	const std::string& group_name,
	const std::string& base_frame,
	const std::vector< std::string >& tip_frames,
	double search_discretization );

virtual void Initialize(
	std::shared_ptr< const JointChain > joint_chain,
	std::shared_ptr< const Mat4d > home_configuration,
	double search_discretization );

[[nodiscard]] bool ForwardKinematic(
	const std::span< const double >& joints,
	geometry_msgs::msg::Pose& pose ) const;

[[nodiscard]] bool ForwardKinematic(
	const VecXd& joints,
	Mat4d& pose ) const;

[[nodiscard]] bool InverseKinematic(
	const geometry_msgs::msg::Pose& target_pose,
	const std::span< const double >& seed_joints,
	std::vector< double >& joints ) const;

protected:
std::shared_ptr< const JointChain > joint_chain_;
std::shared_ptr< const Mat4d > home_configuration_;
std::unique_ptr< WorkspaceFilter > workspace_filter_;

[[nodiscard]] virtual bool InverseKinematic(
	const Mat4d& target_pose,
	const std::span< const double >& seed_joints,
	VecXd& joints ) const = 0;

[[nodiscard]] bool AreValidInitializeParameters(
	const moveit::core::RobotModelConstPtr& robot_model,
	const std::string& group_name,
	const std::string& base_frame,
	const std::vector< std::string >& tip_frames,
	double search_discretization ) const noexcept;

[[nodiscard]] bool CheckLimits( const std::span< const double >& joints ) const;
};
}
