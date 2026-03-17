#include "HybridSolver/WristCenterSolver.hpp"

#include "DLSSolver/NumericSolverResult.hpp"
#include "Global.hpp"
#include "HybridSolver/WristCenterJointsModel.hpp"
#include "Joint/JointChain.hpp"
#include "FABRIKSolver/FabrikSolver.hpp"
#include "SolverResult.hpp"

#include <memory>

namespace SOArm100::Kinematics
{

// ------------------------------------------------------------

WristCenterJointsSolver::WristCenterJointsSolver(
    std::shared_ptr< const JointChain > joint_chain,
    std::shared_ptr< const Mat4d > home_configuration,
    const WristCenterJointsModel& wrist_center_model )
{
    fabrik_solver_ = std::make_unique< FABRIKKinematicsSolver >();
    wrist_center_model_ = std::make_unique< const WristCenterJointsModel >( wrist_center_model );

	const auto& active_joints = joint_chain->GetActiveJoints();
	auto start = active_joints[wrist_center_model.start_index];
	auto end = active_joints[wrist_center_model.start_index + wrist_center_model.count - 1];

    std::shared_ptr< const JointChain > wrist_center_joint_chain;
    std::shared_ptr< const Mat4d > wrist_center_home_configuration;

	if ( start == active_joints.front() && end == active_joints.back() )
	{
		wrist_center_joint_chain = joint_chain;
		wrist_center_home_configuration = home_configuration;
	}
	else
	{
		wrist_center_joint_chain = std::make_shared< const JointChain >( joint_chain->SubChain( start, end ) );
		wrist_center_home_configuration = std::make_shared< const Mat4d >( wrist_center_model.home_configuration );
	}

    fabrik_solver_->Initialize( wrist_center_joint_chain, wrist_center_home_configuration, 0.01 );
}

// ------------------------------------------------------------

SolverResult WristCenterJointsSolver::Heuristic(
    const Mat4d& target,
    const std::span< const double >& seed_joints,
    double search_discretization ) const
{
    assert( seed_joints.size() == GetJointCount() );

    FABRIKKinematicsSolver::SolverParameters params;
    params.error_tolerance = 10 * translation_tolerance;
    params.max_iterations = 10;
    fabrik_solver_->SetParameters( params );

    return ToSolverResult( fabrik_solver_->InverseKinematic( target, seed_joints ) );
}

// ------------------------------------------------------------

SolverResult WristCenterJointsSolver::IK(
    const Mat4d& target,
    const std::span< const double >& seed_joints,
    double search_discretization ) const
{
    assert( seed_joints.size() == GetJointCount() );

    FABRIKKinematicsSolver::SolverParameters params;
    params.error_tolerance = translation_tolerance;
    params.max_iterations = 50;
    fabrik_solver_->SetParameters( params );

    return ToSolverResult( 
        fabrik_solver_->InverseKinematic( target, seed_joints ) );
}

// ------------------------------------------------------------

bool WristCenterJointsSolver::FK( const VecXd& joints, Mat4d& fk ) const
{
    assert( joints.size() == GetJointCount() );
    return fabrik_solver_->ForwardKinematic( joints, fk );
}

// ------------------------------------------------------------

}