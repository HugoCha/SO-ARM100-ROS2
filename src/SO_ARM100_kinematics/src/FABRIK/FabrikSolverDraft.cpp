#include "FABRIK/FabrikSolverDraft.hpp"

#include "Global.hpp"

#include "FABRIK/FabrikAnalyzer.hpp"
#include "Model/IKJointGroupModelBase.hpp"
#include "Model/Joint/JointGroup.hpp"
#include "Model/Joint/JointState.hpp"
#include "Model/Skeleton/Skeleton.hpp"
#include "Model/Skeleton/SkeletonState.hpp"
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

	auto skeleton = model_->GetSkeleton();
	Model::SkeletonState skeleton_state( skeleton );

	Mat4d group_target = ComputeGroupWorldTarget( problem.seed, problem.target );
	const Vec3d p_target = Translation( group_target );
	const Vec3d p_base = skeleton->Articulation( 0 )->Center();

	skeleton_state.SetState( problem.seed );

	std::cout << "Initial State" << std::endl;
	std::cout << "Seed          = " << problem.seed.transpose() << std::endl;
	std::cout << "Target        = " << p_target.transpose() << std::endl;
	std::cout << "Bones  = " << std::endl;
	PrintBones( *model_->GetSkeleton() );

	double error;
	for ( int iter = 0; iter < parameters_.max_iterations; iter++ )
	{
		std::cout << "--------------------- Iteration " << iter << " ---------------------" << std::endl;
		auto bone_states = skeleton_state.GetBoneStates();
		error = Utils::Distance( p_target, bone_states.back().Origin() );

		// std::cout << "joints= " << buffers.joints.transpose() << std::endl;
		// std::cout << "FK    = " << Translation( buffers.fk ).transpose() << std::endl;
		// std::cout << "Error = " << error << std::endl;
		// std::cout << "Pose Error = " << ( p_target - buffers.states.back().Origin() ).transpose() << std::endl;
		// std::cout << "Pose  = " << std::endl;
		// PrintStates( buffers.states );

		if ( error < parameters_.error_tolerance )
		{
			return { IKSolverState::Converged, skeleton_state.GetJointValues(),
			         error, iter };
		}

		if ( context.StopRequested() )
		{
			return { IKSolverState::NotRun, skeleton_state.GetJointValues(),
			         error, iter };
		}

		BackwardPass( p_target, skeleton_state, bone_states );
		// std::cout << "Backward = " << std::endl;
		// PrintStates( buffers.states );
		ForwardPass( p_base, skeleton_state, bone_states );
		//std::cout << "Forward = " << std::endl;
		// PrintStates( buffers.states );
		// std::cout << "Project = " << std::endl;
		//PrintStates( buffers.states );
		// ForwardKinematics( *model_->GetSkeleton(), buffers.states, 0 );
		// std::cout << "FK = " << std::endl;
		// PrintStates( buffers.states );
	}

	return { IKSolverState::MaxIterations, skeleton_state.GetJointValues(),
	         error, parameters_.max_iterations };
}

// ------------------------------------------------------------

void FABRIKSolverDraft::BackwardPass(
	const Vec3d& p_target,
	const Model::SkeletonState& skeleton_state,
	std::vector< Model::BoneState >& bone_states ) const
{
	const int n = bone_states.size();
	Vec3d last_origin = p_target;

	for ( int i = n - 1; i > 0; i-- )
	{
		bone_states[i].Direction() =
	        ( last_origin - bone_states[i].Origin() ).normalized() * bone_states[i].GetBone()->Length();

		skeleton_state.ApplyConstraint( bone_states[i], i );

		bone_states[i].Origin() =
			last_origin - bone_states[i].Direction();

		last_origin = bone_states[i].Origin();
	}
}

// ------------------------------------------------------------

void FABRIKSolverDraft::ForwardPass(
	const Vec3d& p_base,
	Model::SkeletonState& skeleton_state,
	std::vector< Model::BoneState >& bone_states ) const
{
	const int n = bone_states.size();
	bone_states[0].Origin() = p_base;

	for ( int i = 0; i < n; i++ )
	{
	    bone_states[i].Direction() =
	        (bone_states[i+1].Origin() - bone_states[i].Origin()).normalized() * bone_states[i].GetBone()->Length();

		skeleton_state.UpdateValue( bone_states[i], i );

	    bone_states[i+1].Origin() =
			bone_states[i].Origin() + bone_states[i].Direction();
	}
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
