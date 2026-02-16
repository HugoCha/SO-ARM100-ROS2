#include "HybridSolver/BaseNumericWristSolver.hpp"

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
    JointChain joint_chain, 
    const BaseJointModel& base_model, 
    const NumericJointsModel& numeric_model, 
    const WristModel& wrist_model) :
    buffer_( SolverBuffer( 1, numeric_model.count, wrist_model.active_joint_count ) )
{
    base_joint_solver_ = std::make_unique< BaseJointSolver >( joint_chain, base_model );
    numeric_solver_ = std::make_unique< NumericJointsSolver >( joint_chain, numeric_model );
    wrist_solver_ = std::make_unique< WristSolver >( joint_chain, wrist_model );
}

// ------------------------------------------------------------

SolverResult BaseNumericWristSolver::IK(
	const Mat4d& target_pose,
	const std::span< const double >& seed_joints,
    double discretization ) const
{
    SolverResult result( buffer_.Size() );

    wrist_solver_->ComputeWristCenter( target_pose, buffer_.wrist_center );

	if ( ( buffer_.base_result = base_joint_solver_->IK(
			   buffer_.wrist_center,
			   seed_joints,
               discretization ) ).Unreachable() )
    {
        result.state = SolverState::Unreachable;
        return result;
    }

	base_joint_solver_->FK( buffer_.base_result.joints, buffer_.T_base );
	buffer_.num_target = Inverse( buffer_.T_base ) * buffer_.wrist_center;

	if ( ( buffer_.numeric_result = numeric_solver_->IK(
				buffer_.num_target,
				seed_joints,
                discretization ) ).Unreachable() )
    {
        result.state = SolverState::Unreachable;
        return result;
    }

	numeric_solver_->FK( buffer_.numeric_result.joints, buffer_.T_num );
	buffer_.wrist_target = Inverse( buffer_.T_num ) * buffer_.num_target;
	if ( ( buffer_.wrist_result = wrist_solver_->IK(
                buffer_.wrist_target,
				seed_joints,
                discretization ) ).Unreachable() )
    {
        result.state = SolverState::Unreachable;
        return result;
    }

    result.state = GetSolverState( 
    {
        buffer_.base_result, 
        buffer_.numeric_result, 
        buffer_.wrist_result 
    } );
	
    result.joints << buffer_.base_result.joints,
	                 buffer_.numeric_result.joints,
	                 buffer_.wrist_result.joints;

	return true;
}

// ------------------------------------------------------------

}