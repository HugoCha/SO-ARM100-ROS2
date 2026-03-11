#include "HybridSolver/NumericJointsSolver.hpp"

#include "DLSSolver/DLSKinematicsSolver.hpp"
#include "DLSSolver/NumericSolverResult.hpp"
#include "Global.hpp"
#include "HybridSolver/NumericJointsModel.hpp"
#include "SolverType.hpp"

#include <iostream>
#include <memory>

namespace SOArm100::Kinematics
{

// ------------------------------------------------------------

NumericJointsSolver::NumericJointsSolver(
	std::shared_ptr< const JointChain > joint_chain,
	std::shared_ptr< const Mat4d > home_configuration,
	const NumericJointsModel& numeric_joint_model,
	SolverType type )
{
	numeric_joints_model_ = std::make_unique< const NumericJointsModel >( numeric_joint_model );

	const auto& active_joints = joint_chain->GetActiveJoints();
	auto start = active_joints[numeric_joint_model.start_index];
	auto end = active_joints[numeric_joint_model.start_index + numeric_joint_model.count - 1];

	if ( start == active_joints.front() && end == active_joints.back() )
	{
		joint_chain_ = joint_chain;
		home_configuration_ = home_configuration;
	}
	else
	{
		joint_chain_ = std::make_shared< const JointChain >( joint_chain->SubChain( start, end ) );
		home_configuration_ = std::make_shared< const Mat4d >( numeric_joint_model.home_configuration );
	}

	dls_solver_ = std::make_unique< DLSKinematicsSolver >( InitializeParameters( type ) );
	dls_solver_->Initialize(
		joint_chain_,
		home_configuration_,
		0.01 );
}

// ------------------------------------------------------------

NumericJointsSolver::NumericJointsSolver(
	std::shared_ptr< const JointChain > joint_chain,
	std::shared_ptr< const Mat4d > home_configuration ) :
	NumericJointsSolver( 
		joint_chain, 
		home_configuration, 
		{ 0, joint_chain->GetActiveJointCount(), *home_configuration },
		SolverType::Full )
{
}

// ------------------------------------------------------------

DLSKinematicsSolver::SolverParameters NumericJointsSolver::InitializeParameters( SolverType type )
{
	DLSKinematicsSolver::SolverParameters parameters;
	switch ( type ) 
	{
		case SolverType::Orientation:
			parameters.error_tolerance = rotation_tolerance;
			parameters.rotation_weight = 1.0;
			parameters.translation_weight = 0;
			parameters.max_iterations = 50;
			parameters.gradient_tolerance = 10 * gradient_tolerance;
			parameters.min_sv_tolerance = 1e-1;
			parameters.max_stalle_iterations = 5;
			parameters.min_step = 0.05;
			parameters.max_step = 0.5;
			parameters.min_damping = 0.05;
			parameters.max_damping = 0.5;
			break;
		case SolverType::Position:
			parameters.error_tolerance = translation_tolerance;
			parameters.rotation_weight = 0;
			parameters.translation_weight = 1.0;
			parameters.max_iterations = 50;
			parameters.gradient_tolerance = 1e-5;
			parameters.max_stalle_iterations = 5;
			parameters.min_step = 0.01;
			parameters.max_step = 1.0;
			parameters.min_sv_tolerance = 1e-2;
			parameters.min_damping = 5e-3;
			parameters.max_damping = 5e-1;
			parameters.max_dq = 0.5;
			break;
		case SolverType::Full:
			parameters.error_tolerance = error_tolerance;
			parameters.rotation_weight = 1.0;
			parameters.translation_weight = 10.0;
			parameters.max_iterations = 200;
			parameters.gradient_tolerance = 1e-6;
			parameters.max_stalle_iterations = 5;
			parameters.min_step = 0.1;
			parameters.max_step = 1.0;
			parameters.min_sv_tolerance = 1e-1;
			parameters.min_damping = 1e-2;
			parameters.max_damping = 3e-1;
			parameters.max_dq = 1.0;
		break;
	}
	return parameters;
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
	auto result = dls_solver_->InverseKinematic(
							target_pose,
							seed_joints.subspan( numeric_joints_model_->start_index, numeric_joints_model_->count ) );
	std::cout << "Iterations=" << result.iterations_used << std::endl;
	return ToSolverResult( result );
}

// ------------------------------------------------------------

}