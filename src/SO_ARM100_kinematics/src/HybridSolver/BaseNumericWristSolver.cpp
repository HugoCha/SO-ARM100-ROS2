#include "HybridSolver/BaseNumericWristSolver.hpp"

#include "Global.hpp"
#include "HybridSolver/BaseJointSolver.hpp"
#include "HybridSolver/NumericJointsSolver.hpp"
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
	const NumericJointsModel& numeric_model,
	const WristModel& wrist_model ) :
	joint_chain_( joint_chain ),
	home_configuration_( home_configuration ),
	buffer_( SolverBuffer( numeric_model.count, wrist_model.active_joint_count ) )
{
	base_joint_presolver_ = std::make_unique< BaseJointSolver >(
		joint_chain,
		home_configuration,
		base_model );

	numeric_presolver_ = std::make_unique< NumericJointsSolver >(
		joint_chain, 
		home_configuration,
		numeric_model,
		SolverType::Position );

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
	assert( seed_joints.size() == buffer_.Size() );

	SolverResult result( buffer_.Size() );

	buffer_.seed_joints.assign( seed_joints.begin(), seed_joints.end() );

	// Base joint Heuristic for Full solver
	wrist_presolver_->ComputeWristCenter( target_pose, buffer_.wrist_center );

	buffer_.wrist_center_target = ToTransformMatrix(
	Mat3d::Identity(),
	buffer_.wrist_center );
	buffer_.base_result = base_joint_presolver_->Heuristic(
			buffer_.wrist_center_target,
			buffer_.seed_joints,
			discretization );
	buffer_.seed_joints[0] = buffer_.base_result.joints[0];


	// Intermediate joint Heuristic for Full solver
	Mat4d T_base;
	base_joint_presolver_->FK( buffer_.base_result.joints, T_base );
	Mat4d num_target = Inverse( T_base ) * buffer_.wrist_center_target;
	buffer_.numeric_result = numeric_presolver_->IK(
			num_target,
			buffer_.seed_joints,
			discretization );
	std::copy( buffer_.numeric_result.joints.begin(),
			buffer_.numeric_result.joints.begin() + buffer_.numeric_result.joints.size(),
			buffer_.seed_joints.begin() + numeric_presolver_->GetNumericJointsModel()->start_index );

	// Wrist joint Heuristic for Full solver
	numeric_presolver_->FK( buffer_.numeric_result.joints, buffer_.T_num );
	buffer_.wrist_target =  Inverse( buffer_.T_num ) * Inverse( T_base ) * target_pose;
	buffer_.wrist_result = wrist_presolver_->Heuristic(
			buffer_.wrist_target,
			buffer_.seed_joints,
			discretization );

	std::copy( buffer_.wrist_result.joints.begin(),
			buffer_.wrist_result.joints.begin() + buffer_.wrist_result.joints.size(),
			buffer_.seed_joints.begin() + wrist_presolver_->GetWristModel()->active_joint_start );
	
	return full_solver_->IK( target_pose, buffer_.seed_joints, discretization );
}

// ------------------------------------------------------------

}