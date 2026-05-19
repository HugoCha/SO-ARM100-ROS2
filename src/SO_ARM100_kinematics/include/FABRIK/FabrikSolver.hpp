#pragma once

#include "Global.hpp"

#include "Model/IKJointGroupModelBase.hpp"
#include "Model/Joint/JointGroup.hpp"
#include "Model/KinematicModel.hpp"
#include "Model/Skeleton/BoneState.hpp"
#include "Model/Skeleton/Skeleton.hpp"
#include "Model/Skeleton/SkeletonState.hpp"
#include "Solver/IKSolverBase.hpp"

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
	int max_stalled_iterations;
	double error_tolerance;

	SolverParameters( 
		int max_iterations = 50, 
		int max_stalled_iterations = 3,
		double error_tolerance = 5e-3 ) :
		max_iterations( max_iterations ),
		max_stalled_iterations( max_stalled_iterations ),
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
struct SolverHistory
{
int stalled_error_cnt;
double last_non_stalled_error;
VecXd best_joints;
double best_error;
};

SolverParameters parameters_;

std::vector< Model::BoneState >
	ComputeBoneStates( const Model::SkeletonState& skeleton_state ) const;

void BackwardPass(
	const Vec3d& p_target,
	const Model::SkeletonState& skeleton_state,
	std::vector< Model::BoneState >& bone_states ) const;

void ForwardPass(
	const Vec3d& p_base,
	Model::SkeletonState& skeleton_state,
	std::vector< Model::BoneState >& bone_states ) const;

void UpdateValues(
	Model::SkeletonState& skeleton_state,
	std::vector< Model::BoneState >& bone_states ) const;

void UpdateHistory( 
	const VecXd& joints,
	double error,
	SolverHistory& history ) const;

bool HasFixedBaseOrigin() const;
};
}