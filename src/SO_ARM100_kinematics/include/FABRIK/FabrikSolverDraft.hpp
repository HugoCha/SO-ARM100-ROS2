#pragma once

#include "Global.hpp"

#include "Model/IKJointGroupModelBase.hpp"
#include "Model/Joint.hpp"
#include "Model/JointChain.hpp"
#include "Model/JointGroup.hpp"
#include "Model/JointState.hpp"
#include "Model/KinematicModel.hpp"
#include "Model/Skeleton.hpp"
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
        old_states.resize( n_joints + 1 );
		states.resize( n_joints + 1 );
	}

	int GetSize() const {
		return joints.size();
	}
};

class Articulation
{
std::vector< std::pair< int, const Model::Joint* > > joints;
std::pair< int, const Model::Joint* > child;
};

SolverParameters parameters_;
const Model::Skeleton skeleton_;

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

void ProjectJoints(
    const Vec3d& p_target,
    const std::span< Model::JointState >& old_joint_states,
    const std::span< Model::JointState >& joint_states ) const;

void ProjectJoint(
    const Vec3d& p_target,
    const std::span< Model::JointState >& old_joint_states,
    const std::span< Model::JointState >& joint_states,
    int index ) const;

void ProjectPrismaticJoint(
    const Model::Joint* joint,
    const Vec3d& p_target,
    Model::JointState& state ) const;

void ProjectRevoluteJoint(
    const Model::Joint* joint,
    const Vec3d& p_target,
    const Model::JointState& old_state,
    const Model::JointState& new_state,
    const Model::JointState& old_child,
    const Model::JointState& new_child,
    Model::JointState& state ) const;

int ForwardDirectionChidlIndex(
    const Model::JointChain& chain,
    const Model::Skeleton& skeleton,
    int index ) const;

Vec3d ForwardDirection(
    const Model::JointState& parent,
    const Vec3d& p_target,
    const std::span< const Model::JointState >& childs ) const;

int ForwardDirectionChild(
    const Model::JointState& parent,
    const std::span< const Model::JointState >& childs ) const;

void ForwardKinematics(
    const Model::Skeleton& skeleton,
    const std::span< Model::JointState >& states,
    int index ) const;

void UpdateJoints(
    const std::span< const Model::JointState >& states,
    VecXd& joints ) const;

void PrintBones(  const Model::Skeleton& skeleton  ) const;
void PrintStates( const std::span< const Model::JointState >& state ) const;
};
}