#include "HybridSolver/NumericJointsSolver.hpp"

#include "DLSSolver/DLSKinematicsSolver.hpp"
#include "DLSSolver/NumericSolverResult.hpp"
#include "Global.hpp"

#include <memory>

namespace SOArm100::Kinematics
{

// ------------------------------------------------------------

NumericJointsSolver::NumericJointsSolver(
	std::shared_ptr< const JointChain > joint_chain,
	std::shared_ptr< const Mat4d > home_configuration )
{
	dls_solver_ = std::make_unique< DLSKinematicsSolver >();
	dls_solver_->Initialize(
		joint_chain,
		home_configuration,
		0.01 );
}

// ------------------------------------------------------------

bool NumericJointsSolver::FK( const VecXd& joints, Mat4d& fk ) const
{
	return dls_solver_->ForwardKinematic( joints, fk );
}

// ------------------------------------------------------------

SolverResult NumericJointsSolver::IK(
	const Mat4d& target_pose,
	const std::span< const double >& seed_joints,
	double search_discretization ) const
{
	return  ToSolverResult( 
		dls_solver_->InverseKinematic( target_pose, seed_joints ) );
}

// ------------------------------------------------------------

}