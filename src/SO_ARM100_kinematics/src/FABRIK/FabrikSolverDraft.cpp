#include "FABRIK/FabrikSolverDraft.hpp"

#include "Global.hpp"

#include "FABRIK/FabrikAnalyzer.hpp"
#include "Model/IKJointGroupModelBase.hpp"
#include "Model/Joint/JointGroup.hpp"
#include "Model/Skeleton/BoneState.hpp"
#include "Model/Skeleton/Skeleton.hpp"
#include "Model/Skeleton/SkeletonState.hpp"
#include "ModelAnalyzer/SkeletonAnalyzer.hpp"
#include "Solver/IKProblem.hpp"
#include "Solver/IKRunContext.hpp"
#include "Solver/IKSolution.hpp"
#include "Solver/IKSolverState.hpp"
#include "Utils/Distance.hpp"
#include "Utils/KinematicsUtils.hpp"
#include "Utils/StringConverter.hpp"

#include <Eigen/src/Geometry/AngleAxis.h>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
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

	if ( problem.seed.size() != n_joints )
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
	std::cout << *model_->GetSkeleton() << std::endl;
	
	double error;
	for ( int iter = 0; iter < parameters_.max_iterations; iter++ )
	{
		std::cout << "--------------------- Iteration " << iter << " ---------------------" << std::endl;
		std::cout << skeleton_state << std::endl;
		auto bone_states = ComputeBoneStates( skeleton_state );
		auto tip_position = bone_states.back().Origin(); 
		error = Utils::Distance( p_target, tip_position );
		
		std::cout << "joints= " << skeleton_state.GetJointValues().transpose() << std::endl;
		std::cout << "Error = " << error << std::endl;
		std::cout << "Tip position = " << tip_position.transpose() << std::endl;
		std::cout << "Pose Error = " << ( p_target - tip_position ).transpose() << std::endl;

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
		ForwardPass( p_base, skeleton_state, bone_states );
		UpdateValues( skeleton_state, bone_states );
	}

	return { IKSolverState::MaxIterations, skeleton_state.GetJointValues(),
	         error, parameters_.max_iterations };
}

// ------------------------------------------------------------

std::vector< Model::BoneState > FABRIKSolverDraft::ComputeBoneStates( 
	const Model::SkeletonState& skeleton_state ) const
{
	auto bone_states = skeleton_state.GetBoneStates();

	if ( skeleton_state.GetSkeleton()->ArticulationCount() > skeleton_state.GetSkeleton()->BonesCount() )
	{
		auto final_bone_state = Model::BoneState( bone_states.back().Origin() + bone_states.back().Direction(), Vec3d::Zero() );
		bone_states.emplace_back( final_bone_state );
	}

	return bone_states;
}

// ------------------------------------------------------------

void FABRIKSolverDraft::BackwardPass(
	const Vec3d& p_target,
	const Model::SkeletonState& skeleton_state,
	std::vector< Model::BoneState >& bone_states ) const
{
	const int n = bone_states.size() - 1;
	bone_states[n].Origin() = p_target;

	for ( int i = n - 1; i >= 0; i-- )
	{
		bone_states[i].Direction() =
			( bone_states[i+1].Origin() - bone_states[i].Origin() ).normalized() * bone_states[i].GetBone()->Length();

		// std::cout << "Bwd" << std::to_string( i ) << " : " 
		// 	<< bone_states[i] << std::endl;

		skeleton_state.ApplyConstraint( bone_states[i], i );

		bone_states[i].Origin() =
			bone_states[i+1].Origin() - bone_states[i].Direction();

		// std::cout << "Bwd" << std::to_string( i ) << " : " 
		// 		  << bone_states[i] << std::endl;
	}
}

// ------------------------------------------------------------

void FABRIKSolverDraft::ForwardPass(
	const Vec3d& p_base,
	Model::SkeletonState& skeleton_state,
	std::vector< Model::BoneState >& bone_states ) const
{
	const int n = bone_states.size() - 1;
	bone_states[0].Origin() = p_base;

	for ( int i = 0; i < n; i++ )
	{
		bone_states[i].Direction() =
			( bone_states[i + 1].Origin() - bone_states[i].Origin() ).normalized() * bone_states[i].GetBone()->Length();

		skeleton_state.ApplyConstraint( bone_states[i], i );
		// std::cout << "Fwd" << std::to_string( i ) << " : " 
		// 	<< bone_states[i] << std::endl;

		bone_states[i + 1].Origin() =
			bone_states[i].Origin() + bone_states[i].Direction();
	}
}

// ------------------------------------------------------------

void FABRIKSolverDraft::UpdateValues(
	Model::SkeletonState& skeleton_state,
	std::vector< Model::BoneState >& bone_states ) const
{
	const int n = bone_states.size() - 1;

	for ( int i = 0; i < n; i++ )
	{
		//std::cout << std::to_string( i ) << " : "<< bone_states[i] << std::endl;
		skeleton_state.UpdateValue( bone_states[i], i, 1.0 );
	}
	
	//std::cout << std::to_string( i ) << " : "<< bone_states[i] << std::endl;
	bone_states[n].Origin() = bone_states[n-1].Origin() + bone_states[n-1].Direction();
}

// ------------------------------------------------------------

}
