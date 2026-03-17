#include "HybridSolver/NumericWristSolver.hpp"

#include "Global.hpp"
#include "HybridSolver/BaseJointSolver.hpp"
#include "HybridSolver/NumericJointsSolver.hpp"
#include "HybridSolver/WristCenterSolver.hpp"
#include "HybridSolver/WristCenterJointsModel.hpp"
#include "HybridSolver/WristSolver.hpp"
#include "Utils/KinematicsUtils.hpp"
#include "SolverResult.hpp"

#include <memory>

namespace SOArm100::Kinematics
{

// ------------------------------------------------------------

NumericWristSolver::NumericWristSolver(
	std::shared_ptr< const JointChain > joint_chain,
	std::shared_ptr< const Mat4d > home_configuration,
	const WristCenterJointsModel& wrist_center_model,
	const WristModel& wrist_model ) :
	joint_chain_( joint_chain ),
	home_configuration_( home_configuration ),
	buffer_( SolverBuffer( wrist_center_model.count, wrist_model.active_joint_count ) )
{
	wrist_center_solver_ = std::make_unique< WristCenterJointsSolver >( joint_chain, home_configuration, wrist_center_model );
	wrist_solver_ = std::make_unique< WristSolver >( joint_chain, home_configuration, wrist_model );
	numeric_solver_ = std::make_unique< NumericJointsSolver >( joint_chain, home_configuration );
}

// ------------------------------------------------------------

SolverResult NumericWristSolver::IK(
	const Mat4d& target_pose,
	const std::span< const double >& seed_joints,
	double discretization ) const
{
	return false;
}

// ------------------------------------------------------------

}