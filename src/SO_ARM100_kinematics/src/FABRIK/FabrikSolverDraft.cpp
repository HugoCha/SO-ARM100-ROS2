#include "FABRIK/FabrikSolverDraft.hpp"

#include "Global.hpp"

#include "FABRIK/FabrikAnalyzer.hpp"
#include "Model/IKJointGroupModelBase.hpp"
#include "Model/JointGroup.hpp"
#include "Model/JointState.hpp"
#include "Model/Skeleton.hpp"
#include "ModelAnalyzer/SkeletonAnalyzer.hpp"
#include "Solver/IKProblem.hpp"
#include "Solver/IKRunContext.hpp"
#include "Solver/IKSolution.hpp"
#include "Solver/IKSolverState.hpp"
#include "Utils/Distance.hpp"
#include "Utils/KinematicsUtils.hpp"

#include <Eigen/src/Geometry/AngleAxis.h>
#include <ios>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <span>
#include <sstream>
#include <string>
#include <vector>

namespace SOArm100::Kinematics::Solver
{

// ------------------------------------------------------------

static rclcpp::Logger get_logger()
{
	static rclcpp::Logger logger = rclcpp::get_logger( "FabrikKinematicsSolver" );
	return logger;
}

// ------------------------------------------------------------

FABRIKSolverDraft::FABRIKSolverDraft(
	Model::KinematicModelConstPtr model,
	Model::JointGroup group,
	SolverParameters parameters ) :
	Model::IKJointGroupModelBase( model, group ),
	parameters_( parameters )
{
}

// ------------------------------------------------------------

FABRIKSolverDraft::FABRIKSolverDraft(
	Model::KinematicModelConstPtr model,
	SolverParameters parameters ) :
	FABRIKSolverDraft(
		model,
		Model::JointGroup::CreateFromRange( "full", 0, model->GetChain()->GetActiveJointCount(), model->GetHomeConfiguration() ),
		parameters )
{
}

// ------------------------------------------------------------

IKSolution FABRIKSolverDraft::Solve(
	const IKProblem& problem,
	const IKRunContext& context ) const
{
	if ( model_->IsUnreachable( problem.target ) )
	{
		return { IKSolverState::Unreachable, {}};
	}

	const int n_joints = GetChain()->GetActiveJointCount();

	if ( problem.seed.size() < n_joints )
	{
		RCLCPP_ERROR( get_logger(), "InitializeState: Joint vector size mismatch." );
		return { IKSolverState::NotRun, {}};
	}

	SolverBuffers buffers = SolverBuffers( GetGroup().Size() );

	buffers.joints = problem.seed;
	Mat4d group_target = ComputeGroupWorldTarget( problem.seed, problem.target );
	const Vec3d p_target = Translation( group_target );

	ComputeJointStates( buffers.joints,  buffers.states, buffers.fk );
	const Vec3d p_base = buffers.states[0].Origin();

	if ( Utils::Distance( p_base, p_target ) > model_->GetSkeleton()->TotalLength() )
	{
		return { IKSolverState::Unreachable, {}};
	}

	std::cout << "Initial State" << std::endl;
	std::cout << "Seed          = " << problem.seed.transpose() << std::endl;
	std::cout << "Target        = " << p_target.transpose() << std::endl;
	std::cout << "Bones  = " << std::endl;
	PrintBones( *model_->GetSkeleton() );

	double error;
	for ( int iter = 0; iter < parameters_.max_iterations; iter++ )
	{
		std::cout << "--------------------- Iteration " << iter << " ---------------------" << std::endl;
		error = Utils::Distance( p_target, buffers.states.back().Origin() );

		std::cout << "joints= " << buffers.joints.transpose() << std::endl;
		std::cout << "FK    = " << Translation( buffers.fk ).transpose() << std::endl;
		std::cout << "Error = " << error << std::endl;
		std::cout << "Pose Error = " << ( p_target - buffers.states.back().Origin() ).transpose() << std::endl;
		std::cout << "Pose  = " << std::endl;
		PrintStates( buffers.states );

		if ( error < parameters_.error_tolerance )
		{
			return { IKSolverState::Converged, buffers.joints,
			         error, iter };
		}

		if ( context.StopRequested() )
		{
			return { IKSolverState::NotRun, buffers.joints,
			         error, iter };
		}

		buffers.old_states = buffers.states;

		BackwardPass( p_target, *model_->GetSkeleton(), buffers.states );
		std::cout << "Backward = " << std::endl;
		PrintStates( buffers.states );
		ForwardPass( p_base, *model_->GetSkeleton(), buffers.states );
		std::cout << "Forward = " << std::endl;
		PrintStates( buffers.states );
		std::cout << "Project = " << std::endl;
		PrintStates( buffers.states );
		ForwardKinematics( *model_->GetSkeleton(), buffers.states, 0 );
		std::cout << "FK = " << std::endl;
		PrintStates( buffers.states );
	}

	return { IKSolverState::MaxIterations,  buffers.joints,
	         error, parameters_.max_iterations };
}

// ------------------------------------------------------------

void FABRIKSolverDraft::ComputeJointStates(
	const VecXd& joints,
	std::vector< Model::JointState >& states,
	Mat4d& fk ) const
{
	// const int n = GetGroup().Size();

	// ComputeGroupWorldJointStatesFK( joints, states, fk );

	// states[n].pose.origin.noalias() = Translation( fk );
	// states[n].pose.axis.setZero();
	// states[n].value = 0;
}

// ------------------------------------------------------------

std::vector< Vec3d > FABRIKSolverDraft::ComputeBones(
	Model::KinematicModelConstPtr model,
	Model::JointGroup group )
{
	const int n_joints = model->GetChain()->GetActiveJointCount();
	const int n_group_joints = group.LastIndex() + 1;

	std::vector< Mat4d > joint_poses( n_joints );
	std::vector< Vec3d > bones( n_group_joints + 1 );
	Mat4d fk;

	model->GetChain()->ComputeJointPosesFK(
		VecXd::Zero( n_group_joints ),
		group.tip_home,
		joint_poses,
		fk );

	for ( int i = 0; i < n_group_joints - 1; i++ )
	{
		bones[i] = Translation( joint_poses[i + 1] ) - Translation( joint_poses[i] );

		if ( bones[i].norm() < epsilon )
			bones[i] = Vec3d::Zero();
	}

	bones[n_group_joints] = Translation( fk ) - Translation( joint_poses[n_group_joints - 1] );

	return bones;
}

// ------------------------------------------------------------

void FABRIKSolverDraft::BackwardPass(
	const Vec3d& target,
	const Model::Skeleton& skeleton,
	const std::span< Model::JointState >& states ) const
{
	// const int n = states.size() - 1;
	// states[n].pose.origin = target;

	// for ( int i = n - 1; i > 0; i-- )
	// {
	//     Vec3d dir =
	//         (states[i].pose.origin - states[i+1].pose.origin).normalized();

	//     states[i].pose.origin =
	//         states[i+1].pose.origin + dir * skeleton.Length( i );
	// }
}

// ------------------------------------------------------------

void FABRIKSolverDraft::ForwardPass(
	const Vec3d& p_base,
	const Model::Skeleton& skeleton,
	const std::span< Model::JointState >& states ) const
{
	// const int n = states.size() - 1;
	// states[0].pose.origin = p_base;

	// for ( int i = 0; i < n; i++ )
	// {
	//     Vec3d dir =
	//         (states[i+1].pose.origin - states[i].pose.origin).normalized();

	//     states[i+1].pose.origin =
	//         states[i].pose.origin + dir * skeleton.Length( i );
	// }
}

// ------------------------------------------------------------

void FABRIKSolverDraft::ForwardKinematics(
	const Model::Skeleton& skeleton,
	const std::span< Model::JointState >& states,
	int index ) const
{
	// const int n = states.size();
	// Quaternion world_rot = Quaternion::Identity();

	// const auto* last_joint = GetActiveJoint( index );
	// for ( int i = index; i < n - 2; i++ )
	// {
	//     Vec3d bone;

	//     if ( last_joint->GetType() == Model::JointType::REVOLUTE )
	//     {
	//         auto rot = AngleAxis( states[i].value, last_joint->Axis() );
	//         world_rot = world_rot * rot;
	//         bone = world_rot * skeleton.Direction( i );
	//     }
	//     else if ( last_joint->GetType() == Model::JointType::PRISMATIC )
	//     {
	//         bone = world_rot * ( skeleton.Direction( i ) + states[i].value * last_joint->Axis() );
	//     }

	//     if ( i < n - 2 )
	//     {
	//         auto joint = GetActiveJoint( i + 1 );

	//         states[i+1].pose.axis =
	//             world_rot * joint->Axis();

	//         last_joint = joint;
	//     }

	//     states[i+1].pose.origin =
	//         states[i].pose.origin + bone;
	// }
}

// ------------------------------------------------------------

void FABRIKSolverDraft::PrintBones( const Model::Skeleton& skeleton ) const
{
	const int n_bones = skeleton.Bones().size();
	std::stringstream bones_ss;
	for ( int i = 0; i < n_bones; i++ )
	{
		bones_ss << "{ ";
		bones_ss << "dir = " << skeleton.Direction( i ).transpose();
		bones_ss << ", length = " << skeleton.Length( i );
		bones_ss << " }" << std::endl;
	}
	std::cout << bones_ss.str();
}

// ------------------------------------------------------------

void FABRIKSolverDraft::PrintStates( const std::span< const Model::JointState >& states ) const
{
	const int n_poses = states.size();
	std::stringstream poses_ss;
	poses_ss << std::fixed << std::setprecision( 3 ) << std::showpos;
	for ( int i = 0; i < n_poses; i++ )
	{
		poses_ss << "{ origin = ";
		poses_ss << std::internal << states[i].Origin().transpose();
		poses_ss << ", axis = ";
		poses_ss << std::right << states[i].Axis().transpose();
		poses_ss << ", value = ";
		poses_ss << states[i].Value();
		poses_ss << " }" << std::endl;
	}
	std::cout << poses_ss.str();
}

// ------------------------------------------------------------

}
