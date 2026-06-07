#pragma once

#include "Global.hpp"

#include "PipelineSolver/PipelineSolver.hpp"
#include "Model/KinematicModel.hpp"
#include "PipelineSolver/PipelineSolverParameters.hpp"

#include <moveit/kinematics_base/kinematics_base.hpp>
#include <geometry_msgs/geometry_msgs/msg/pose.hpp>
#include <memory>
#include <moveit/macros/class_forward.hpp>
#include <vector>

namespace moveit::core
{
MOVEIT_CLASS_FORWARD( JointModel );
MOVEIT_CLASS_FORWARD( JointModelGroup );
MOVEIT_CLASS_FORWARD( RobotModel );
}

namespace SOArm100::Kinematics
{

class RobotArmKinematicsSolver
{
public:
RobotArmKinematicsSolver();
virtual ~RobotArmKinematicsSolver() = default;

virtual bool Initialize(
	const moveit::core::RobotModelConstPtr& robot_model,
	const std::string& group_name,
	const std::string& base_frame,
	const std::vector< std::string >& tip_frames,
	double search_discretization );

virtual bool Initialize(
	Model::KinematicModelConstPtr model,
	std::unique_ptr< Solver::PipelineSolver > solver,
	double search_discretization );

Solver::PipelineSolverParameters& Parameters(){
	return solver_->Parameters();
}

const Solver::PipelineSolverParameters& Parameters() const {
	return solver_->Parameters();
}

Model::KinematicModelConstPtr GetModel() const {
	return model_;
}

[[nodiscard]] bool ForwardKinematic(
	const std::vector< double >& joints,
	geometry_msgs::msg::Pose& pose ) const;

[[nodiscard]] bool ForwardKinematic(
	const VecXd& joints,
	Mat4d& pose ) const;

[[nodiscard]] bool InverseKinematic(
	const geometry_msgs::msg::Pose& target_pose,
	const std::span< const double >& seed_joints,
	const std::span< const double >& consistency_limits,
	long timeout_ms,
	std::vector< double >& joints ) const;

[[nodiscard]] bool InverseKinematic(
	const Mat4d& target,
	const VecXd& seed,
	const VecXd& consistency,
	long timeout_ms,
	VecXd& joints ) const;

private:
Model::KinematicModelConstPtr model_;
std::unique_ptr< Solver::PipelineSolver > solver_;

[[nodiscard]] bool InverseKinematic(
	const Mat4d& target,
	const VecXd& seed,
	const VecXd& consistency,
	long timeout_ms,
	double* joints,
	int n_joints ) const;

[[nodiscard]] bool AreValidInitializeParameters(
	const moveit::core::RobotModelConstPtr& robot_model,
	const std::string& group_name,
	const std::string& base_frame,
	const std::vector< std::string >& tip_frames,
	double search_discretization ) const noexcept;

void InitializeSolver( const Model::KinematicModelConstPtr& model );
};
}
