#include "Heuristic/WristHeuristic.hpp"

#include "Global.hpp"

#include "Heuristic/IKHeuristicState.hpp"
#include "Heuristic/IKPresolution.hpp"
#include "Model/IKJointGroupModelBase.hpp"
#include "Model/Joint/JointGroup.hpp"
#include "Model/KinematicTopology/WristTopology.hpp"
#include "Solver/IKProblem.hpp"
#include "SphericalSolver/SphericalModel.hpp"
#include "SphericalSolver/SphericalSolver.hpp"
#include "SphericalSolver/SphericalSolution.hpp"
#include "UniversalSolver/UniversalSolution.hpp"
#include "Utils/KinematicsUtils.hpp"

#include <cassert>
#include <memory>
#include <stdexcept>

namespace SOArm100::Kinematics::Heuristic
{

// ------------------------------------------------------------

WristHeuristic::WristHeuristic(
	Model::KinematicModelConstPtr model,
	Model::JointGroup wrist_group ) :
	Model::IKJointGroupModelBase( model, wrist_group ),
	spherical_solver_( nullptr )
{
	if ( GetWristTopology() == Model::WristTopology::Revolute2 )
	{
		auto universal_model = Model::UniversalModel::ComputeModel(
			model->GetChain()->GetActiveJoint( wrist_group.Index( 0 ) ),
			model->GetChain()->GetActiveJoint( wrist_group.Index( 1 ) ) );

		if ( !universal_model )
		{
			throw std::invalid_argument( "Wrist is not Universal" );
		}

		Solver::UniversalSolver::SolverParameters params;
		universal_solver_ = std::make_unique< Solver::UniversalSolver >(
			*universal_model,
			params );
	}
	else if ( GetWristTopology() == Model::WristTopology::Revolute3 )
	{
		auto spherical_model = Model::SphericalModel::ComputeModel(
			model->GetChain()->GetActiveJoint( wrist_group.Index( 0 ) ),
			model->GetChain()->GetActiveJoint( wrist_group.Index( 1 ) ),
			model->GetChain()->GetActiveJoint( wrist_group.Index( 2 ) ) );

		if ( !spherical_model )
		{
			throw std::invalid_argument( "Wrist is not Spherical" );
		}

		Solver::SphericalSolver::SolverParameters params;
		spherical_solver_ = std::make_unique< Solver::SphericalSolver >(
			*spherical_model,
			params );
	}
}

// ------------------------------------------------------------

Model::WristTopology WristHeuristic::GetWristTopology() const
{
	if ( GetGroup().indices.size() > 3 )
		return Model::WristTopology::None;
	return static_cast< Model::WristTopology >( GetGroup().indices.size() );
}

// ------------------------------------------------------------

IKPresolution WristHeuristic::Presolve(
	const Solver::IKProblem& problem,
	const Solver::IKRunContext& context ) const
{
	Mat4d wrist_center = ComputeWristCenter( problem.seed,  problem.target );
	Mat4d wrist_target = Inverse( wrist_center ) * problem.target;
	Mat3d R_wrist_target = Rotation( wrist_target );

	switch ( GetWristTopology() )
	{
	case Model::WristTopology::Revolute1:
		return SolveRevolute1( problem, R_wrist_target );
	case Model::WristTopology::Revolute2:
		return SolveRevolute2( problem, R_wrist_target );
	case Model::WristTopology::Revolute3:
		return SolveRevolute3( problem, R_wrist_target );
	default:
		break;
	}

	return { problem.seed, IKHeuristicState::PartialSuccess };
}

// ------------------------------------------------------------

Mat4d WristHeuristic::ComputeWristCenter(
	const VecXd& seed,
	const Mat4d& target ) const
{
	Mat4d wrist_center;

	Mat4d wrist_center_home =
		model_->GetHomeConfiguration() * Inverse( GetGroup().tip_home );

	if ( GetAncestor() )
	{
		VecXd ancestor_joints = GetAncestor()->GetGroupJoints( seed );
		model_->GetChain()->ComputeFK(
			ancestor_joints,
			wrist_center_home,
			wrist_center );
	}
	else
	{
		wrist_center = wrist_center_home;
	}

	return wrist_center;
}

// ------------------------------------------------------------

IKPresolution WristHeuristic::SolveRevolute1( const Solver::IKProblem& problem, const Mat3d& R_target ) const
{
	IKPresolution presolution { problem.seed, IKHeuristicState::Fail };
	VecXd wrist_solution = GetGroup().GetGroupJoints( problem.seed );

	const auto* joint = GetActiveJoint( GetGroup().Index( 0 ) );
	const Vec3d& axis = joint->Axis();

	AngleAxis aa( R_target );
	double proj = aa.axis().dot( axis );

	// aa.axis() // axis <=> | proj | = 1
	if ( std::abs( proj ) < 1 - epsilon )
	{
		wrist_solution[0] = joint->GetLimits().Clamp( wrist_solution[0] );
		presolution.state = IKHeuristicState::PartialSuccess;
	}
	else
	{
		wrist_solution[0] = joint->GetLimits().Clamp( proj * aa.angle() );
		presolution.state = IKHeuristicState::Success;
	}

	GetGroup().SetGroupJoints( wrist_solution, presolution.joints );
	return presolution;
}

// // ------------------------------------------------------------

IKPresolution WristHeuristic::SolveRevolute2( const Solver::IKProblem& problem, const Mat3d& R_target ) const
{
	// IKPresolution presolution { problem.seed, IKHeuristicState::Fail };
	// VecXd wrist_solution = GetGroup().GetGroupJoints( problem.seed );

	// const Vec3d& e1 = GetActiveJoint( GetGroup().Index( 0 )  )->Axis();
	// const Vec3d& e2 = GetActiveJoint( GetGroup().Index( 1 )  )->Axis();

	// Vec3d v  = e2;
	// Vec3d vp = R_target * e2;

	// Vec3d v_perp  = v  - e1.dot( v )  * e1;
	// Vec3d vp_perp = vp - e1.dot( vp ) * e1;

	// if ( v_perp.norm() < epsilon || vp_perp.norm() < epsilon )
	// {
	// 	presolution.state = IKHeuristicState::PartialSuccess;
	// 	return presolution;
	// }

	// double q1 = atan2(
	// 	e1.dot( v_perp.cross( vp_perp ) ),
	// 	v_perp.dot( vp_perp ) );

	// Mat3d R1 = AngleAxis( q1, e1 ).toRotationMatrix();
	// Mat3d R2 = R1.transpose() * R_target;

	// // R2 = I + sin(q2) e2x + ( 1 - cos(q2) ) * ( e2x * e2x )
	// // Transpose( e2x ) = -e2x
	// // R2 - Transpose( R2 ) = 2 * sin( q2 )
	// // trace( R ) = 1 + 2 * cos( q2 )
	// Vec3d w = 0.5 * Vec3d(
	// 	R2( 2, 1 ) - R2( 1, 2 ),
	// 	R2( 0, 2 ) - R2( 2, 0 ),
	// 	R2( 1, 0 ) - R2( 0, 1 ) );

	// double q2 = atan2( e2.dot( w ), ( R2.trace() - 1.0 ) * 0.5 );

	// Mat3d R_check =
	// 	AngleAxis( q1, e1 ).toRotationMatrix() *
	// 	AngleAxis( q2, e2 ).toRotationMatrix();

	// Mat3d Rerr = R_check.transpose() * R_target;
	// AngleAxis aa_err( Rerr );

	// // std::cout << "seed 1 = " << wrist_solution[0] << " seed 2 = " << wrist_solution[1] << std::endl;
	// // std::cout << "q1 = " << q1 << " q2 = " << q2 << std::endl;
	// // std::cout << "target =\n" << R_target << std::endl;
	// // std::cout << "result =\n" << R_check << std::endl;
	// // std::cout << "Rerror  =" << Rerr << std::endl;
	// // std::cout << "AAerror  =" << aa_err.angle() * aa_err.axis() << std::endl;
	// // std::cout << "Other Axis =" << e1.cross( e2 ) << std::endl;

	// if ( RotationError( R_target, R_check ) > rotation_tolerance )
	// {
	// 	presolution.state = IKHeuristicState::PartialSuccess;
	// }
	// else
	// {
	// 	presolution.state = IKHeuristicState::Success;
	// }

	// wrist_solution[0] = q1;
	// wrist_solution[1] = q2;
	// GetGroup().SetGroupJoints( wrist_solution, presolution.joints );
	// return presolution;

	assert( universal_solver_ );
	IKPresolution presolution;
	presolution.joints = problem.seed;

	auto wrist_seed = GetGroup().GetGroupJoints( problem.seed );
	auto universal_solution = universal_solver_->SolveFromRotation( R_target, wrist_seed, problem.tolerance );

	presolution.error = universal_solution.fk_error;
	presolution.state = universal_solution.reachable ? IKHeuristicState::Success : IKHeuristicState::PartialSuccess;
	GetGroup().SetGroupJoints( universal_solution.angles, presolution.joints );

	return presolution;
}

// ------------------------------------------------------------

IKPresolution WristHeuristic::SolveRevolute3( const Solver::IKProblem& problem, const Mat3d& R_target ) const
{
	assert( spherical_solver_ );
	IKPresolution presolution;
	presolution.joints = problem.seed;

	auto wrist_seed = GetGroup().GetGroupJoints( problem.seed );
	auto spherical_solution = spherical_solver_->SolveFromRotation( R_target, wrist_seed, problem.tolerance );

	presolution.error = spherical_solution.fk_error;
	presolution.state = spherical_solution.reachable ? IKHeuristicState::Success : IKHeuristicState::PartialSuccess;
	GetGroup().SetGroupJoints( spherical_solution.angles, presolution.joints );

	return presolution;
}

// ------------------------------------------------------------

// IKPresolution WristHeuristic::SolveRevolute3( const VecXd& seed, const Mat3d& R_target ) const
// {
// 	IKPresolution presolution { seed, IKHeuristicState::Fail };
// 	VecXd wrist_solution = GetGroup().GetGroupJoints( seed );

// 	const Vec3d& e1 = GetActiveJoint( GetGroup().Index( 0 ) )->Axis();
// 	const Vec3d& e2 = GetActiveJoint( GetGroup().Index( 1 ) )->Axis();
// 	const Vec3d& e3 = GetActiveJoint( GetGroup().Index( 2 ) )->Axis();

// 	Mat3d R = R_target;

// 	// STEP 1: Solve q1 by aligning e3
// 	Vec3d v  = e3;
// 	Vec3d vp = R * e3;

// 	Vec3d v_perp  = v  - e1.dot( v )  * e1;
// 	Vec3d vp_perp = vp - e1.dot( vp ) * e1;

// 	if ( v_perp.norm() < epsilon || vp_perp.norm() < epsilon )
// 	{
// 		presolution.state = IKHeuristicState::PartialSuccess;
// 		return presolution;
// 	}

// 	double q1 = atan2( e1.dot( v_perp.cross( vp_perp ) ), v_perp.dot( vp_perp ) );

// 	Mat3d R1 = AngleAxis( q1, e1 ).toRotationMatrix();
// 	R = R1.transpose() * R;

// 	// STEP 2: Solve q2 by aligning e3 again
// 	v  = e3;
// 	vp = R * e3;

// 	v_perp  = v  - e2.dot( v )  * e2;
// 	vp_perp = vp - e2.dot( vp ) * e2;

// 	if ( v_perp.norm() < epsilon || vp_perp.norm() < epsilon )
// 	{
// 		wrist_solution << q1;
// 		presolution.state = IKHeuristicState::PartialSuccess;
// 		GetGroup().SetGroupJoints( wrist_solution, presolution.joints );
// 		return presolution;
// 	}

// 	double q2 = atan2( e2.dot( v_perp.cross( vp_perp ) ), v_perp.dot( vp_perp ) );

// 	Mat3d R2 = AngleAxis( q2, e2 ).toRotationMatrix();
// 	R = R2.transpose() * R;

// 	// STEP 3: Extract q3 directly
// 	Vec3d w = 0.5 * Vec3d(
// 		R( 2, 1 ) - R( 1, 2 ),
// 		R( 0, 2 ) - R( 2, 0 ),
// 		R( 1, 0 ) - R( 0, 1 ) );

// 	double q3 = atan2( e3.dot( w ), ( R.trace() - 1.0 ) * 0.5 );

// 	Mat3d R_check =
// 		AngleAxis( q1, e1 ).toRotationMatrix() *
// 		AngleAxis( q2, e2 ).toRotationMatrix() *
// 		AngleAxis( q3, e3 ).toRotationMatrix();

// 	if ( !R_check.isApprox( R_target, rotation_tolerance ) )
// 	{
// 		wrist_solution << q1, q2;
// 		presolution.state = IKHeuristicState::PartialSuccess;
// 	}
// 	else
// 	{
// 		wrist_solution << q1, q2, q3;
// 		presolution.state = IKHeuristicState::Success;
// 	}

// 	GetGroup().SetGroupJoints( wrist_solution, presolution.joints );
// 	return presolution;
// }

// ------------------------------------------------------------

}