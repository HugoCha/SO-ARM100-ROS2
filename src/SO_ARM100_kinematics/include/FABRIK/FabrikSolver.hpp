#pragma once

#include "Global.hpp"

#include "Model/IKJointGroupModelBase.hpp"
#include "Model/JointGroup.hpp"
#include "Model/KinematicModel.hpp"
#include "Model/Pose.hpp"
#include "Solver/IKSolverBase.hpp"

#include <span>
#include <vector>

namespace SOArm100::Kinematics::Solver
{
class FABRIKSolver : 
	public Model::IKJointGroupModelBase, 
	public IKSolverBase
{
public:
struct SolverParameters
{
	int max_iterations;
	double error_tolerance;

	SolverParameters( int max_iterations = 20, double error_tolerance = 5e-3 ) :
		max_iterations( max_iterations ),
		error_tolerance( error_tolerance )
	{
	}
};

explicit FABRIKSolver(
	Model::KinematicModelConstPtr model,
	SolverParameters parameters = SolverParameters() );

explicit FABRIKSolver(
	Model::KinematicModelConstPtr model,
	Model::JointGroup joint_group,
	SolverParameters parameters = SolverParameters() );

~FABRIKSolver() = default;

FABRIKSolver( const FABRIKSolver& ) = delete;
FABRIKSolver& operator = ( const FABRIKSolver& ) = delete;

[[nodiscard]] SolverParameters GetParameters() const {
	return parameters_;
}

void SetParameters( const SolverParameters& parameters ){
	parameters_ = parameters;
}

[[nodiscard]] virtual Solver::IKSolution Solve(
	const IKProblem& problem,
	const IKRunContext& context ) const override;

private:
struct SolverBuffers
{
	int fabrik_start_idx { 0 };
	std::vector< double > bone_lengths;
	std::vector< Model::Pose > old_poses;
	std::vector< Model::Pose > poses;
	VecXd joints;
	Mat4d fk;

	explicit SolverBuffers( int n_joints )
	{
		joints.resize( n_joints );
		old_poses.resize( n_joints + 1 );
		poses.resize( n_joints + 1 );
		bone_lengths.resize( n_joints );
	}

	int GetSize() const {
		return joints.size();
	}
};

SolverParameters parameters_;

static Model::JointGroup ComputeFabrikGroup( 
	Model::KinematicModelConstPtr model, 
	std::optional< Model::JointGroup > sub_group );

void PreSolveAzimuthJoints(
	const Vec3d& p_target,
	int& fabrik_start_idx,
	VecXd& joints
	) const;

void ComputePoses(
	const VecXd& joints,
	std::vector< Model::Pose >& poses,
	Mat4d& fk ) const;

void ComputeBoneLengths(
	const std::span< const Model::Pose >& poses,
	const std::span< double >& bone_lengths ) const;

double TotalBoneLength( const std::span< const double > bone_lengths ) const;

void ForwardPass(
	const Vec3d& target,
	const std::span< const double >& bone_lengths,
	int start_idx,
	const std::span< Model::Pose >& poses ) const;

void BackwardPass(
	const Vec3d& base,
	const std::span< const double >& bone_lengths,
	int start_idx,
	const std::span< Model::Pose >& poses ) const;

double RevolutePositionToAngle(
	const Model::Pose& parent_pose,
	const Model::Pose& old_child_pose,
	const Model::Pose& new_child_pose,
	double current_angle ) const;

void ApplyRevoluteJointLimit(
	const Model::Pose&  parent_pose,
	const Model::Limits& limits,
	double& current_angle,
	const std::span< Model::Pose >& child_poses ) const;

double PrismaticPositionToDisplacement(
	const Model::Pose& old_pose,
	const Model::Pose& new_pose,
	double current_displacement ) const;

void UpdateJointValues(
	const std::span< const Model::Pose >& old_poses,
	int start_idx,
	const std::span< Model::Pose >& poses,
	VecXd& joints ) const;

void PrintBoneLengths( const std::span< const double > bone_lengths ) const;
void PrintPoses( const std::span< const Model::Pose >& poses ) const;
};
}