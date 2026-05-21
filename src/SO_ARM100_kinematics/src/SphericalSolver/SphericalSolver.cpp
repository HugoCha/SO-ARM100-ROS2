#include "SphericalSolver/SphericalSolver.hpp"

#include "Global.hpp"

#include "SphericalSolver/SphericalModel.hpp"
#include "SphericalSolver/SphericalSolution.hpp"
#include "SphericalSolver/SphericalSolutionBranch.hpp"
#include "Utils/KinematicsUtils.hpp"
#include <cmath>
#include <limits>

namespace SOArm100::Kinematics::Solver
{

// ------------------------------------------------------------

SphericalSolver::SphericalSolver( const Model::SphericalModel& model, SolverParameters parameters ) :
    model_( model ),
    parameters_( parameters )
{
}

// ------------------------------------------------------------

SphericalSolution SphericalSolver::SolveFromRotation(
	const Mat3d& R_target,
    std::optional< Vec3d > theta_pref ) const
{
    SphericalSolution out{};

	const auto& branches = model_.Decompose( R_target );

	SphericalSolutionBranch best_branch;
    best_branch.cost = std::numeric_limits<double>::infinity();

	const Vec3d prefered = theta_pref.value_or( Vec3d(
        model_.GetJoint( 0 )->GetLimits().Center(),
        model_.GetJoint( 1 )->GetLimits().Center(),
        model_.GetJoint( 2 )->GetLimits().Center() ) );


	for ( auto branch : branches )
	{
		const Vec3d& angles = branch;
		double cost = 0.0;
        
        cost += DeviationCost( prefered, angles );
		cost += LimitViolationCost( angles, 1e6 );
		cost += RotationErrorCost( R_target, angles );

        if ( cost < best_branch.cost )
        {
            best_branch.angles = angles;
            best_branch.cost = cost;
            best_branch.phi = 0.0;
        }
	}

	bool all_in = CheckLimits( best_branch.angles );

	const Mat3d R_fk      = model_.Recompose( best_branch.angles );
	const double fk_error = RotationError( R_target, R_fk );

	out.angles             = best_branch.angles;
	out.phi                = best_branch.phi;
	out.cost               = best_branch.cost;
	out.fk_error           = fk_error;
	out.singularity_margin = model_.SingularityMargin( R_target );
	out.reachable          = all_in && fk_error < parameters_.error_tol;
	out.near_singular      = model_.IsSingular( R_target, parameters_.singularity_tol );

	return out;
}

// ------------------------------------------------------------

SphericalSolution SphericalSolver::SolveFromTwoVectors( 	
	const Vec3d& p_tcp_local,
	const Vec3d& p_target ) const
{
	SphericalSolution out{};

	const double r_local  = p_tcp_local.norm();
	const double r_target = p_target.norm();

	if ( r_local < 1e-9 )
	{
		out.reachable = false;
		return out;
	}

    const Vec3d& p_tcp_local_c = p_tcp_local.normalized();
    const Vec3d& p_target_c = p_target.normalized();

    const Mat3d R_base = Eigen::Quaterniond::FromTwoVectors( p_tcp_local_c, p_target_c ).toRotationMatrix();

    const Vec3d free_axis = p_target_c;

	auto cost_fn =
		[&]( double limit_violation_penalty, double fk_error_violation_penalty, double phi ) -> SphericalSolutionBranch
		{
			const Mat3d R_phi  = Eigen::AngleAxisd( phi, free_axis ).toRotationMatrix();
			const Mat3d R_canonical = R_phi * R_base;

			const auto& branches = model_.Decompose( R_canonical );

            SphericalSolutionBranch best_local{};
            best_local.cost = std::numeric_limits<double>::infinity();

            for ( const auto& angles : branches )
            {
                double cost = 0.0;

                cost += LimitViolationCost( angles, limit_violation_penalty );
                cost += FKErrorCost( p_tcp_local_c, p_target_c, angles ) * fk_error_violation_penalty;

                if ( cost < best_local.cost )
                {
                    best_local.angles = angles;
                    best_local.cost   = cost;
                    best_local.phi    = phi;
                }
            }

            return best_local;
		};

	auto infinite_penalty_cost = [&]( double phi ){ 
		return cost_fn( std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), phi ); 
	};

	auto best_coarse_branch = FirstSolutionSearch( infinite_penalty_cost );
	
	if ( std::isinf( best_coarse_branch.cost ) )
	{
		auto least_violation_penalty_cost = [&]( double phi ){ 
			return cost_fn( 1e6, 1.0, phi ); 
		};
		best_coarse_branch = GridSearch( least_violation_penalty_cost );
	}

	const Mat3d R_opt  = Eigen::AngleAxisd( best_coarse_branch.phi, free_axis ) * R_base;

	bool all_in = CheckLimits( best_coarse_branch.angles );

	const Mat3d R_fk      = model_.Recompose( best_coarse_branch.angles );
	const Vec3d p_fk      = R_fk * p_tcp_local;
	const double fk_error = ( p_fk - p_target ).norm();

	out.angles             = best_coarse_branch.angles;
	out.phi                = best_coarse_branch.phi;
	out.cost               = best_coarse_branch.cost;
	out.fk_error           = fk_error;
	out.singularity_margin = model_.SingularityMargin( R_opt );
	out.reachable          = all_in && fk_error < parameters_.error_tol;;
	out.near_singular      = model_.IsSingular( R_opt, parameters_.singularity_tol );

	return out;
}

// ------------------------------------------------------------

SphericalSolution SphericalSolver::SolveAndOptimizeFromTwoVectors(
	const Vec3d& p_tcp_local,
	const Vec3d& p_target,
	std::optional< Vec3d > theta_pref ) const
{
	SphericalSolution out{};

	const double r_local  = p_tcp_local.norm();
	const double r_target = p_target.norm();

	if ( r_local < 1e-9 )
	{
		out.reachable = false;
		return out;
	}

    const Vec3d& p_tcp_local_c = p_tcp_local.normalized();
    const Vec3d& p_target_c = p_target.normalized();

    Mat3d R_base = Eigen::Quaterniond::FromTwoVectors( p_tcp_local_c, p_target_c ).toRotationMatrix();
    
    const Vec3d free_axis = p_target_c;

	const Vec3d prefered = theta_pref.value_or( Vec3d(
													model_.GetJoint( 0 )->GetLimits().Center(),
													model_.GetJoint( 1 )->GetLimits().Center(),
													model_.GetJoint( 2 )->GetLimits().Center() ) );

	const double dphi = 2.0 * M_PI / static_cast< double >( parameters_.phi_samples );

	auto cost_fn =
		[&]( double phi ) -> SphericalSolutionBranch
		{
			const Mat3d R_phi  = Eigen::AngleAxisd( phi, free_axis ).toRotationMatrix();
			const Mat3d R_canonical = R_phi * R_base;

			const auto& branches = model_.Decompose( R_canonical );

            SphericalSolutionBranch best_local{};
            best_local.cost = std::numeric_limits<double>::infinity();

            for ( const auto& angles : branches )
            {
                double cost = 0.0;

                cost += DeviationCost( prefered, angles );
                cost += LimitViolationCost( angles, parameters_.limit_penalty );
                cost += SingularityCost( R_canonical );
                cost += FKErrorCost( p_tcp_local_c, p_target_c, angles );

                if ( cost < best_local.cost )
                {
                    best_local.angles = angles;
                    best_local.cost   = cost;
                    best_local.phi    = phi;
                }
            }

            return best_local;
		};

	auto best_coarse_branch = GridSearch( cost_fn );
	auto best_branch = GoldenSearchSection(
		cost_fn,
		best_coarse_branch.phi - dphi,
		best_coarse_branch.phi + dphi );
    
	const Mat3d R_opt  = Eigen::AngleAxisd( best_branch.phi, free_axis ) * R_base;

	bool all_in = CheckLimits( best_branch.angles );

	const Mat3d R_fk      = model_.Recompose( best_branch.angles );
	const Vec3d p_fk      = R_fk * p_tcp_local;
	const double fk_error = ( p_fk - p_target ).norm();

	out.angles             = best_branch.angles;
	out.phi                = best_branch.phi;
	out.cost               = best_branch.cost;
	out.fk_error           = fk_error;
	out.singularity_margin = model_.SingularityMargin( R_opt );
	out.reachable          = all_in && fk_error < parameters_.error_tol;;
	out.near_singular      = model_.IsSingular( R_opt, parameters_.singularity_tol );

	return out;
}

// ------------------------------------------------------------

double SphericalSolver::DeviationCost( const Vec3d& prefered, const Vec3d& angles ) const
{
	return ( prefered - angles ).squaredNorm();
}

// ------------------------------------------------------------

double SphericalSolver::LimitViolationCost( 
    const Vec3d& angles, 
    double violation_weight ) const
{
	double cost = 0.0;

	for ( int i = 0; i < 3; ++i )
	{
		const double lo = model_.GetJoint( i )->GetLimits().Min();
		const double hi = model_.GetJoint( i )->GetLimits().Max();

		if ( angles[i] < lo )
			cost += violation_weight * ( lo - angles[i] ) * ( lo - angles[i] );
		else if ( angles[i] > hi )
			cost += violation_weight * ( angles[i] - hi ) * ( angles[i] - hi );
	}

	return cost;
}

// ------------------------------------------------------------

double SphericalSolver::SingularityCost( const Mat3d& R_canonical ) const
{
	const double margin = model_.SingularityMargin( R_canonical );

	if ( margin < parameters_.singularity_tol )
		return parameters_.singularity_penalty / ( margin * margin + 1e-6 );

	return 0.0;
}

// ------------------------------------------------------------

double SphericalSolver::RotationErrorCost( 
    const Mat3d& R_target, 
    const Vec3d& angles ) const
{
    Mat3d R_fk = model_.Recompose( angles );
    return std::abs( RotationError( R_target, R_fk ) );
}


// ------------------------------------------------------------

double SphericalSolver::FKErrorCost( 
    const Vec3d& p_tcp, 
    const Vec3d& p_target, 
    const Vec3d& angles ) const
{
    const Mat3d R_fk      = model_.Recompose( angles );
	const Vec3d p_fk      = R_fk * p_tcp;
	return ( p_fk - p_target ).norm();
}

// ------------------------------------------------------------

SphericalSolutionBranch SphericalSolver::GridSearch( const CostFn& f ) const
{
	SphericalSolutionBranch best_branch;

	best_branch.phi  = 0.0;
	best_branch.cost = std::numeric_limits< double >::infinity();

	const double dphi = 2.0 * M_PI / static_cast< double >( parameters_.phi_samples );

	for ( int k = 0; k < parameters_.phi_samples; ++k )
	{
		const double phi  = k * dphi;
		const auto& branch    = f( phi );
		if ( branch.cost < best_branch.cost )
		{
			best_branch = branch;
		}
	}

	return best_branch;
}

// ------------------------------------------------------------

SphericalSolutionBranch SphericalSolver::FirstSolutionSearch( const CostFn& f ) const
{
	SphericalSolutionBranch first_branch;

	first_branch.phi  = 0.0;
	first_branch.cost = std::numeric_limits< double >::infinity();

	const double dphi = 2.0 * M_PI / static_cast< double >( parameters_.phi_samples );

	for ( int k = 0; k < parameters_.phi_samples; ++k )
	{
		const double phi  = k * dphi;
		const auto& branch    = f( phi );
		if ( branch.cost < first_branch.cost )
		{
			first_branch = branch;
			return first_branch;
		}
	}

	return first_branch;
}

// ------------------------------------------------------------

SphericalSolutionBranch SphericalSolver::GoldenSearchSection(
	const CostFn& f,
	double a,
	double b,
	double tol ) const
{
	const double gr = ( 1 + sqrt( 5 ) ) / 2.0;  // golden ratio
	double c = b - ( b - a ) / gr;
	double d = a + ( b - a ) / gr;
	for ( int i = 0; i < parameters_.refine_max_iters && std::abs( b - a ) > tol; ++i )
	{
		if ( f( c ).cost < f( d ).cost )
			b = d;
		else
			a = c;
		c = b - ( b - a ) / gr;
		d = a + ( b - a ) / gr;
	}

	return f( 0.5 * ( a + b ) );
}

// ------------------------------------------------------------

bool SphericalSolver::CheckLimits( Vec3d& angles ) const
{
	bool all_in = true;
	for ( int i = 0; i < 3; ++i )
	{
		const auto& limit = model_.GetJoint( i )->GetLimits();
		if ( !limit.Within( angles[i] ) )
		{
			all_in = false;
		}
		angles[i] = limit.Clamp( angles[i] );
	}
	return all_in;
}

// ------------------------------------------------------------

}