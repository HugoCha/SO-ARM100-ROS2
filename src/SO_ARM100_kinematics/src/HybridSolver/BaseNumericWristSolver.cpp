#include "HybridSolver/BaseNumericWristSolver.hpp"

#include "DLSSolver/DLSKinematicsSolver.hpp"
#include "DLSSolver/NumericSolverResult.hpp"
#include "Global.hpp"
#include "HybridSolver/BaseJointSolver.hpp"
#include "HybridSolver/NumericJointsSolver.hpp"
#include "HybridSolver/WristSolver.hpp"
#include "Utils/KinematicsUtils.hpp"
#include "SolverResult.hpp"

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
	buffer_( SolverBuffer( 1, numeric_model.count, wrist_model.active_joint_count ) )
{
	base_joint_presolver_ = std::make_unique< BaseJointSolver >( joint_chain, home_configuration, base_model );
	numeric_presolver_ = std::make_unique< NumericJointsSolver >( joint_chain, home_configuration, numeric_model );
	wrist_presolver_ = std::make_unique< WristSolver >( joint_chain, home_configuration, wrist_model );

	full_solver_ = std::make_unique< DLSKinematicsSolver >();
}

// ------------------------------------------------------------

SolverResult BaseNumericWristSolver::IK(
	const Mat4d& target_pose,
	const std::span< const double >& seed_joints,
	double discretization ) const
{
	SolverResult result( buffer_.Size() );

	buffer_.seed_joints.assign( seed_joints.begin(), seed_joints.end() );

	// Base joint Heuristic for Full solver
	wrist_presolver_->ComputeWristCenter( target_pose, buffer_.wrist_center );
	
	buffer_.base_result = base_joint_presolver_->Heuristic(
				buffer_.wrist_center,
				seed_joints,
				discretization );
	buffer_.seed_joints[0] = buffer_.base_result.joints[0];

	// Intermediate joint Heuristic for Full solver
	buffer_.numeric_result = numeric_presolver_->IK(
			   buffer_.wrist_center,
			   seed_joints,
			   discretization );
	std::copy( buffer_.numeric_result.joints.data(), 
				buffer_.numeric_result.joints.data() + buffer_.numeric_result.joints.size(), 
			  buffer_.seed_joints.data() + numeric_presolver_->GetNumericJointsModel()->start_index );
	
	// Wrist joint Heuristic for Full solver
	numeric_presolver_->FK( buffer_.numeric_result.joints, buffer_.T_num );
	buffer_.wrist_target = Inverse( buffer_.T_num ) * target_pose;
	buffer_.wrist_result = wrist_presolver_->Heuristic(
			   buffer_.wrist_target,
			   seed_joints,
			   discretization );
	std::copy( buffer_.wrist_result.joints.data(), 
				buffer_.wrist_result.joints.data() + buffer_.wrist_result.joints.size(), 
				buffer_.seed_joints.data() + wrist_presolver_->GetWristModel()->active_joint_start );

	return ToSolverResult( full_solver_->InverseKinematic( target_pose, buffer_.seed_joints ) );
}

// ------------------------------------------------------------

}