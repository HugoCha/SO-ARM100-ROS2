#pragma once

#include "Global.hpp"

#include "Model/IKJointGroupModelBase.hpp"
#include "Model/Joint/JointChain.hpp"
#include "Model/Joint/JointGroup.hpp"
#include "Model/Joint/JointState.hpp"
#include "Model/KinematicModel.hpp"
#include "Model/Skeleton/Skeleton.hpp"
#include "Solver/IKSolverBase.hpp"

#include <vector>

namespace SOArm100::Kinematics::Solver
{
class FABRIKSolverDraft :
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

explicit FABRIKSolverDraft(
	Model::KinematicModelConstPtr model,
	SolverParameters parameters = SolverParameters() );

explicit FABRIKSolverDraft(
	Model::KinematicModelConstPtr model,
	Model::JointGroup joint_group,
	SolverParameters parameters = SolverParameters() );

~FABRIKSolverDraft() = default;

FABRIKSolverDraft( const FABRIKSolverDraft& ) = delete;
FABRIKSolverDraft& operator = ( const FABRIKSolverDraft& ) = delete;

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
	std::vector< Model::JointState > old_states;
	std::vector< Model::JointState > states;
	VecXd joints;
	Mat4d fk;

	explicit SolverBuffers( int n_joints )
	{
		joints.resize( n_joints );
		// old_states.resize( n_joints + 1 );
		// states.resize( n_joints + 1 );
	}

	int GetSize() const {
		return joints.size();
	}
};

SolverParameters parameters_;

void ComputeJointStates(
	const VecXd& joints,
	std::vector< Model::JointState >& states,
	Mat4d& fk ) const;

static std::vector< Vec3d > ComputeBones(
	Model::KinematicModelConstPtr model,
	Model::JointGroup group );

void BackwardPass(
	const Vec3d& p_target,
	const Model::Skeleton& skeleton,
	const std::span< Model::JointState >& states ) const;

void ForwardPass(
	const Vec3d& p_base,
	const Model::Skeleton& skeleton,
	const std::span< Model::JointState >& states ) const;

void ForwardKinematics(
	const Model::Skeleton& skeleton,
	const std::span< Model::JointState >& states,
	int index ) const;

void PrintBones(  const Model::Skeleton& skeleton  ) const;
void PrintStates( const std::span< const Model::JointState >& state ) const;
};
}