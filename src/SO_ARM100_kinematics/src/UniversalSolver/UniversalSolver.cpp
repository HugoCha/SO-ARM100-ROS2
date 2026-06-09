#include "UniversalSolver/UniversalSolver.hpp"

#include "Global.hpp"

#include "UniversalSolver/UniversalSolution.hpp"
#include "Utils/KinematicsUtils.hpp"

#include <algorithm>
#include <cstdlib>

namespace SOArm100::Kinematics::Solver
{

// ------------------------------------------------------------

UniversalSolver::UniversalSolver( const Model::UniversalModel& model, SolverParameters parameters ) :
	model_( model ),
	parameters_( parameters )
{
}

// ------------------------------------------------------------

UniversalSolution UniversalSolver::SolveFromRotation(
	const Mat3d& R_target,
	std::optional< Vec2d > theta_pref,
	double tolerance ) const
{
	auto j0 = model_.GetJoint( 0 );
	auto j1 = model_.GetJoint( 1 );

	const auto& l0 = j0->GetLimits();
	const auto& l1 = j1->GetLimits();

	const Vec2d pref = theta_pref.value_or( Vec2d( l0.Center(), l1.Center() ) );

	auto solutions = model_.Decompose( R_target );

	auto cost_fn = [&]( const UniversalSolution& s ) -> double
				   {
					   double cost = 0.0;
					   cost += FKErrorCost( s.fk_error, parameters_.fk_error_penalty, tolerance );
					   cost += DeviationCost( pref, s.angles );
					   cost += LimitCost( s.angles );
					   cost += RotationErrorCost( R_target, s.angles, parameters_.fk_error_penalty, tolerance );
					   return cost;
				   };

	std::sort( solutions.begin(), solutions.end(),
	           [&]( const Solver::UniversalSolution& a, const Solver::UniversalSolution& b ){
			return cost_fn( a ) < cost_fn( b );
		} );

	return solutions[0];
}

// ------------------------------------------------------------

UniversalSolution UniversalSolver::SolveFromTwoVectors(
	const Vec3d& p_tcp_local,
	const Vec3d& p_target,
	std::optional< Vec2d > theta_pref,
	double tolerance ) const
{
	auto j0 = model_.GetJoint( 0 );
	auto j1 = model_.GetJoint( 1 );

	const auto& l0 = j0->GetLimits();
	const auto& l1 = j1->GetLimits();

	const Vec2d pref = theta_pref.value_or( Vec2d( l0.Center(), l1.Center() ) );

	const Vec3d& b0 = p_tcp_local;
	const Vec3d& b1 = p_target;

	auto solutions = model_.Decompose( b0, b1 );

	auto cost_fn = [&]( const UniversalSolution& s ) -> double
				   {
					   double cost = 0.0;
					   cost += FKErrorCost( s.fk_error, parameters_.fk_error_penalty, tolerance );
					   cost += DeviationCost( pref, s.angles );
					   cost += LimitCost( s.angles );
					   return cost;
				   };

	std::sort( solutions.begin(), solutions.end(),
	           [&]( const Solver::UniversalSolution& a, const Solver::UniversalSolution& b ){
			return cost_fn( a ) < cost_fn( b );
		} );

	return solutions[0];
}

// ------------------------------------------------------------

double UniversalSolver::FKErrorCost( 
	double fk_error,	
	double fk_error_penalty, 
	double tolerance ) const
{
	return std::abs( fk_error ) > tolerance ? fk_error_penalty * std::abs( fk_error ) : std::abs( fk_error ) / M_PI;
}

// ------------------------------------------------------------

double UniversalSolver::DeviationCost( const Vec2d& prefered, const Vec2d& angles ) const
{
	auto j0 = model_.GetJoint( 0 );
	auto j1 = model_.GetJoint( 1 );

	const auto& l0 = j0->GetLimits();
	const auto& l1 = j1->GetLimits();

	Vec2d span = Vec2d( l0.Span(), l1.Span() );

	return ( prefered - angles ).norm() / span.norm();
}

// ------------------------------------------------------------

double UniversalSolver::LimitCost( const Vec2d& angles ) const
{
	auto j0 = model_.GetJoint( 0 );
	auto j1 = model_.GetJoint( 1 );

	const auto& l0 = j0->GetLimits();
	const auto& l1 = j1->GetLimits();

	Vec2d center = Vec2d( l0.Center(), l1.Center() );
	Vec2d span = Vec2d( l0.Span(), l1.Span() ) / 2.0;

	return ( center - angles ).norm() / span.norm();
}

// ------------------------------------------------------------

double UniversalSolver::RotationErrorCost(  
	const Mat3d& R_target, 
	const Vec2d& angles,  
	double fk_error_penalty, 
	double tolerance ) const
{
	Mat3d R = model_.Recompose( angles );
	double Rerror = RotationError( R_target, R );
	return std::abs( Rerror ) > tolerance ? fk_error_penalty * std::abs( Rerror ) : std::abs( Rerror );
}

// ------------------------------------------------------------

}