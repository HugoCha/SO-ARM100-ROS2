#include "HybridSolver/BaseNumericWristSolver.hpp"

#include "Global.hpp"
#include "HybridSolver/BaseJointSolver.hpp"
#include "HybridSolver/NumericJointsSolver.hpp"
#include "HybridSolver/WristCenterJointsModel.hpp"
#include "HybridSolver/WristCenterSolver.hpp"
#include "HybridSolver/WristSolver.hpp"
#include "Utils/Converter.hpp"
#include "Utils/KinematicsUtils.hpp"
#include "SolverResult.hpp"

#include <cassert>
#include <memory>

namespace SOArm100::Kinematics
{

// ------------------------------------------------------------

BaseNumericWristSolver::BaseNumericWristSolver(
	std::shared_ptr< const JointChain > joint_chain,
	std::shared_ptr< const Mat4d > home_configuration,
	const BaseJointModel& base_model,
	const WristCenterJointsModel& wrist_center_model,
	const WristModel& wrist_model ) :
	joint_chain_( joint_chain ),
	home_configuration_( home_configuration )
{
	base_joint_presolver_ = std::make_unique< BaseJointSolver >(
		joint_chain,
		home_configuration,
		base_model );

	wrist_center_presolver_ = std::make_unique< WristCenterJointsSolver >(
		joint_chain, 
		home_configuration,
		wrist_center_model );

	wrist_presolver_ = std::make_unique< WristSolver >(
		joint_chain,
		home_configuration,
		wrist_model );

	full_solver_ = std::make_unique< NumericJointsSolver >(
		joint_chain,
		home_configuration );
}

// ------------------------------------------------------------

SolverResult BaseNumericWristSolver::IK(
	const Mat4d& target_pose,
	const std::span< const double >& seed_joints,
	double discretization ) const
{
	Vec3d wrist_center;
	Mat4d wrist_center_pose;
	wrist_presolver_->ComputeWristCenter( target_pose, wrist_center );
	wrist_center_pose = ToTransformMatrix( wrist_center );

	// Base joint Heuristic for Full solver
	auto base_seed = seed_joints.subspan(
		base_joint_presolver_->GetJointStartIndex(),
		base_joint_presolver_->GetJointCount() );
	
	auto base_result = base_joint_presolver_->IK(
		wrist_center_pose,
		base_seed,
		discretization );

	if ( base_result.Unreachable() )
	{
		return { SolverState::Unreachable, {} };
	}

	// Intermediate joint Heuristic for Full solver
	Mat4d T_base;
	base_joint_presolver_->FK( base_result.joints, T_base );
	Mat4d T_wrist_center = Inverse( T_base ) * wrist_center_pose;

	auto wrist_center_seed = seed_joints.subspan(
		wrist_center_presolver_->GetJointStartIndex(),
		wrist_center_presolver_->GetJointCount() );

	auto wrist_center_result = wrist_center_presolver_->Heuristic(
		T_wrist_center,
		wrist_center_seed,
		discretization );
	
	if ( wrist_center_result.Unreachable() )
	{
		return { SolverState::Unreachable, {} };
	}

	// Wrist joint Heuristic for Full solver
	Mat4d T_wrist_orientation =  Inverse( T_wrist_center ) * Inverse( T_base ) * target_pose;
	
	auto wrist_orientation_seed = seed_joints.subspan( 
		wrist_presolver_->GetJointStartIndex(),
		wrist_presolver_->GetJointCount() );

	auto wrist_result = wrist_presolver_->Heuristic(
			T_wrist_orientation,
			wrist_orientation_seed,
			discretization );

	auto heuristic_joints = GetHeuristicJoints( 
		base_result, 
		wrist_center_result, 
		wrist_result );

	return full_solver_->IK( 
		target_pose, 
		heuristic_joints, 
		discretization );
}

// ------------------------------------------------------------

std::vector< double > BaseNumericWristSolver::GetHeuristicJoints( 
	const SolverResult& base_result,
	const SolverResult& wrist_center_result,
	const SolverResult& wrist_result ) const
{
	const int n_joints = base_result.joints.size() + 
						 wrist_center_result.joints.size() + 
						 wrist_result.joints.size();

	std::vector< double > heuristic_joints( n_joints );

	if ( base_result.Success() )
	{
		std::ranges::copy( 
			base_result.joints.begin(), 
			base_result.joints.end(), 
			heuristic_joints.begin() + base_joint_presolver_->GetJointStartIndex() );
	}

	std::ranges::copy(
		wrist_center_result.joints.begin(),
		wrist_center_result.joints.end(),
		heuristic_joints.begin() + wrist_center_presolver_->GetJointStartIndex() );

	std::ranges::copy(
		wrist_result.joints.begin(),
		wrist_result.joints.end(),
		heuristic_joints.begin() + wrist_presolver_->GetJointStartIndex() );
	
	return heuristic_joints;
}

// ------------------------------------------------------------

}