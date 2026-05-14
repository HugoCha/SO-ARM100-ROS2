#include "Euler/SphericalSolver.hpp"

#include "Global.hpp"

#include <cmath>
#include <limits>

namespace SOArm100::Kinematics::Solver
{

// ------------------------------------------------------------

SphericalSolver::SphericalSolver( const Model::EulerModel& model, SolverParameters parameters ) :
	model_( model ),
	parameters_( parameters )
{
}

// ------------------------------------------------------------

SphericalSolver::IKResult SphericalSolver::Solve(
	const Vec3d& p_tcp_local,
	const Vec3d& p_target,
	std::optional< Vec3d > theta_pref ) const
{
	IKResult out{};
	out.reachable = false;

	const double r_local  = p_tcp_local.norm();
	const double r_target = p_target.norm();

	if ( r_local < 1e-9 )
		throw std::runtime_error(
				  "TCP offset is zero — spherical wrist has no position DOF." );

	if ( std::abs( r_local - r_target ) > parameters_.reachability_tol )
		return out;

	out.reachable = true;

	const Vec3d p_tcp_c    = model_.ToCanonical( p_tcp_local );
	const Vec3d p_target_c = model_.ToCanonical( p_target );

	const Vec3d pref = theta_pref.value_or( Vec3d(
												model_.GetJoint( 0 )->GetLimits().Center(),
												model_.GetJoint( 1 )->GetLimits().Center(),
												model_.GetJoint( 2 )->GetLimits().Center() ) );

	const Mat3d R_base = Eigen::Quaterniond::FromTwoVectors( p_tcp_c, p_target_c ).toRotationMatrix();

	const Vec3d free_axis = p_target_c.normalized();

	auto cost_fn = [&]( double phi ) -> double {
		// Compose: rotate about target axis by φ, then apply base rotation
		const Mat3d R_phi  = Eigen::AngleAxisd( phi, free_axis )
							.toRotationMatrix();
		const Mat3d R_cand = R_phi * R_base; // canonical rotation

		// Decompose → physical joint angles (no back-transform needed)
		const Vec3d angles = model_.DecomposeCanonical( R_cand );

		double cost = 0.0;

		// Soft: deviation from preferred pose
		cost += ( angles - pref ).squaredNorm();

		// Hard: joint limit violation (quadratic penalty)
		for ( int i = 0; i < 3; ++i )
		{
			const double lo = model_.GetJoint( i )->GetLimits().Min();
			const double hi = model_.GetJoint( i )->GetLimits().Max();
			if ( angles[i] < lo )
				cost += parameters_.limit_penalty * ( lo - angles[i] ) * ( lo - angles[i] );
			else if ( angles[i] > hi )
				cost += parameters_.limit_penalty * ( angles[i] - hi ) * ( angles[i] - hi );
		}

		// Soft: singularity avoidance
		const double margin = model_.SingularityMargin( R_cand );
		if ( margin < parameters_.singularity_tol )
			cost += parameters_.singularity_penalty / ( margin * margin + 1e-6 );

		return cost;
	};

	// ── 6. Coarse grid search over φ ∈ [0, 2π) ───────────────────────
	double best_phi  = 0.0;
	double best_cost = std::numeric_limits< double >::infinity();
	const double dphi = 2.0 * M_PI / static_cast< double >( parameters_.phi_samples );

	for ( int k = 0; k < parameters_.phi_samples; ++k )
	{
		const double phi  = k * dphi;
		const double c    = cost_fn( phi );
		if ( c < best_cost )
		{
			best_cost = c; best_phi = phi;
		}
	}

	// ── 7. Golden-section refinement in [best_phi ± dphi] ─────────────
	best_phi  = GoldenSearchSection( cost_fn,
	                                 best_phi - dphi,
	                                 best_phi + dphi,
	                                 1e-9,
	                                 parameters_.refine_max_iters );
	best_cost = cost_fn( best_phi );

	// ── 8. Reconstruct solution ───────────────────────────────────────
	const Mat3d R_opt  = Eigen::AngleAxisd( best_phi, free_axis ) * R_base;

	// Physical joint angles from EulerModel decomposition
	const Vec3d angles = model_.DecomposeCanonical( R_opt );

	// Clamp to limits and record which joints violated
	Vec3d clamped;
	bool all_in = true;
	for ( int i = 0; i < 3; ++i )
	{
		const auto& limit = model_.GetJoint( i )->GetLimits();
		if ( !limit.Within( angles[i] ) )
		{
			all_in = false;
		}
		clamped[i] = limit.Clamp( angles[i] );
	}

	// FK verification: recompose and check position error
	const Mat3d R_fk      = model_.RecomposePhysical( clamped );
	const Vec3d p_fk      = R_fk * p_tcp_local;
	const double fk_error = ( p_fk - p_target ).norm();

	out.angles             = clamped;
	out.phi                = best_phi;
	out.cost               = best_cost;
	out.fk_error           = fk_error;
	out.singularity_margin = model_.SingularityMargin( R_opt );
	out.in_limits          = all_in;
	out.near_singular      = model_.IsSingular( R_opt, parameters_.singularity_tol );

	return out;
}

// ------------------------------------------------------------

double SphericalSolver::GoldenSearchSection(
	const std::function< double( double ) >& f,
	double a,
	double b,
	double tol,
	int max_iter )
{
	const double gr = ( 1 + sqrt( 5 ) ) / 2.0;  // golden ratio
	double c = b - ( b - a ) / gr;
	double d = a + ( b - a ) / gr;
	for ( int i = 0; i < max_iter && std::abs( b - a ) > tol; ++i )
	{
		if ( f( c ) < f( d ) )
			b = d;
		else
			a = c;
		c = b - ( b - a ) / gr;
		d = a + ( b - a ) / gr;
	}
	return 0.5 * ( a + b );
}

// ------------------------------------------------------------

}