#pragma once

#include "Global.hpp"

#include "HybridSolver/HybridSolver.hpp"
#include "Model/KinematicModel.hpp"

#include <geometry_msgs/geometry_msgs/msg/pose.hpp>
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
virtual ~KinematicsSolver() = default;

virtual void Initialize(
	const moveit::core::RobotModelConstPtr& robot_model,
	const std::string& group_name,
	const std::string& base_frame,
	const std::vector< std::string >& tip_frames,
	double search_discretization );

virtual void Initialize(
	Model::KinematicModelConstPtr model,
	const Solver::HybridSolver& solver,
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

[[nodiscard]] bool InverseKinematic(
	const Mat4d& target,
	const VecXd& seed,
	VecXd& joints ) const;

private:
Model::KinematicModelConstPtr model_;
std::unique_ptr< const Solver::HybridSolver > solver_;

[[nodiscard]] bool InverseKinematic(
	const Mat4d& target,
	const VecXd& seed,
	double* joints,
	int n_joints ) const;

[[nodiscard]] bool AreValidInitializeParameters(
	const moveit::core::RobotModelConstPtr& robot_model,
	const std::string& group_name,
	const std::string& base_frame,
	const std::vector< std::string >& tip_frames,
	double search_discretization ) const noexcept;
};
}
