#include "Euler/SphericalSolver.hpp"

#include "Global.hpp"

#include <cmath>
#include <limits>
#include <vector>

namespace SOArm100::Kinematics::Solver
{

// ------------------------------------------------------------

SphericalSolver::SphericalSolver( 
	const Model::EulerModel& model, 
	SolverParameters parameters ) :
	model_( model ),
	parameters_( parameters )
{
}

// ------------------------------------------------------------

Mat3d SphericalSolver::ComputeRMinimal(
	const Vec3d& old_dir,
	const Vec3d& new_dir ) const
{
    const double cos_angle = old_dir.dot( new_dir );

    // ── normal case ──────────────────────────────────────────────────────────
    if ( cos_angle > -1.0 + 1e-6 )
        return Quaternion::FromTwoVectors( old_dir, new_dir ).toRotationMatrix();

    // ── antiparallel: new_dir == -old_dir ────────────────────────────────────
    // All valid 180° rotation axes lie in the plane perpendicular to old_dir.
    // Build two orthogonal vectors spanning that plane, then sweep the full
    // circle — this guarantees we find the global best axis, not just one
    // of a few heuristic candidates.

    // Two guaranteed-perpendicular basis vectors for the plane
    Vec3d u = old_dir.unitOrthogonal();           // first basis vector
    Vec3d v = old_dir.cross( u ).normalized();    // second basis vector

    auto score = [&]( const Vec3d& axis ) -> double
    {
        Mat3d R       = AngleAxis( M_PI, axis ).toRotationMatrix();
        auto branches = model_.DecomposeCanonical( R );

        std::vector< EulerBranch > euler_branches;

        for ( int i = 0; i < 2; ++i )
        {
			const Vec3d& angles = branches[i]; 
			double cost = 0.0;
			
			cost += DeviationCost( Vec3d::Zero(), angles );
			cost += LimitViolationCost( angles );
			cost += SingularityCost( R );
	
			euler_branches[i].angles = angles;
			euler_branches[i].cost = cost;
        }

        return euler_branches[0].cost < euler_branches[1].cost ?
			euler_branches[0].cost : euler_branches[1].cost;
    };

    // ── coarse sweep over full circle ─────────────────────────────────────────
    double best_score = std::numeric_limits< double >::max();
    double best_phi   = 0.0;

    for ( int i = 0; i < parameters_.phi_samples; ++i )
    {
        double phi  = 2.0 * M_PI * i / parameters_.phi_samples;
        Vec3d  axis = std::cos( phi ) * u + std::sin( phi ) * v;
        double s    = score( axis );
        if ( s < best_score )
        {
            best_score = s;
            best_phi   = phi;
        }
    }

    // ── refine with golden section around best_phi ────────────────────────────
    double span = 2.0 * M_PI / parameters_.phi_samples;
    double lo   = best_phi - span;
    double hi   = best_phi + span;

    const double phi_golden = ( std::sqrt( 5.0 ) - 1.0 ) / 2.0;
    double c = hi - phi_golden * ( hi - lo );
    double d = lo + phi_golden * ( hi - lo );
    double fc = score( std::cos( c ) * u + std::sin( c ) * v );
    double fd = score( std::cos( d ) * u + std::sin( d ) * v );

    while ( std::abs( hi - lo ) > 1e-8 )
    {
        if ( fc < fd ) { hi = d; d = c; fd = fc; c = hi - phi_golden * ( hi - lo ); fc = score( std::cos( c ) * u + std::sin( c ) * v ); }
        else           { lo = c; c = d; fc = fd; d = lo + phi_golden * ( hi - lo ); fd = score( std::cos( d ) * u + std::sin( d ) * v ); }
    }

    Vec3d best_axis = ( std::cos( ( lo + hi ) / 2.0 ) * u
                      + std::sin( ( lo + hi ) / 2.0 ) * v ).normalized();

    return AngleAxis( M_PI, best_axis ).toRotationMatrix();
}

// ------------------------------------------------------------

SphericalSolver::IKResult SphericalSolver::Solve(
	const Vec3d& p_tcp_local,
	const Vec3d& p_target,
	std::optional< Vec3d > theta_pref ) const
{
	IKResult out{};

	const double r_local  = p_tcp_local.norm();
	const double r_target = p_target.norm();

	if ( r_local < 1e-9 )
	{
		out.reachable = false;
		return out;
	}

	const Vec3d p_tcp_c    = model_.ToCanonical( p_tcp_local );
	const Vec3d p_target_c = model_.ToCanonical( p_target );

	const Vec3d prefered = theta_pref.value_or( Vec3d(
												model_.GetJoint( 0 )->GetLimits().Center(),
												model_.GetJoint( 1 )->GetLimits().Center(),
												model_.GetJoint( 2 )->GetLimits().Center() ) );

	const Mat3d R_base = ComputeRMinimal( p_tcp_c, p_target_c );//Eigen::Quaterniond::FromTwoVectors( p_tcp_c, p_target_c ).toRotationMatrix();
	//const Mat3d R_base = Eigen::Quaterniond::FromTwoVectors( p_tcp_c, p_target_c ).toRotationMatrix();
	
	const Vec3d free_axis = p_target_c.normalized();
	const double dphi = 2.0 * M_PI / static_cast< double >( parameters_.phi_samples );

	auto cost_fn = [&]( double phi ) -> EulerBranch {
		const Mat3d R_phi  = Eigen::AngleAxisd( phi, free_axis ).toRotationMatrix();
		const Mat3d R_canonical = R_phi * R_base;
	
		const auto& branches = model_.DecomposeCanonical( R_canonical );
	
		std::vector< EulerBranch > euler_branches( 2 );
	
		for ( int i = 0; i < 2; i++ )
		{
			const Vec3d& angles = branches[i]; 
			double cost = 0.0;
			
			cost += DeviationCost( prefered, angles );
			cost += LimitViolationCost( angles );
			cost += SingularityCost( R_canonical );
	
			euler_branches[i].angles = angles;
			euler_branches[i].cost = cost;
			euler_branches[i].phi = phi;
		}
	
		return euler_branches[0].cost < euler_branches[1].cost ?
			euler_branches[0] : euler_branches[1];
	};

	auto best_coarse_branch = GridSearch( cost_fn );
	auto best_branch = GoldenSearchSection( 
		cost_fn, 
		best_coarse_branch.phi - dphi, 
		best_coarse_branch.phi + dphi );

	const Mat3d R_opt  = Eigen::AngleAxisd( best_branch.phi, free_axis ) * R_base;

	bool all_in = CheckLimits( best_branch.angles );

	const Mat3d R_fk      = model_.RecomposePhysical( best_branch.angles );
	const Vec3d p_fk      = R_fk * p_tcp_local;
	const double fk_error = ( p_fk - p_target ).norm();

	out.angles             = best_branch.angles;
	out.phi                = best_branch.phi;
	out.cost               = best_branch.cost;
	out.fk_error           = fk_error;
	out.singularity_margin = model_.SingularityMargin( R_opt );
	out.reachable          = all_in;
	out.near_singular      = model_.IsSingular( R_opt, parameters_.singularity_tol );

	return out;
}

// ------------------------------------------------------------

double SphericalSolver::DeviationCost( const Vec3d& prefered, const Vec3d& angles ) const
{
	return ( prefered - angles ).squaredNorm();
}

// ------------------------------------------------------------

double SphericalSolver::LimitViolationCost( const Vec3d& angles ) const
{
	double cost = 0.0;

	for ( int i = 0; i < 3; ++i )
	{
		const double lo = model_.GetJoint( i )->GetLimits().Min();
		const double hi = model_.GetJoint( i )->GetLimits().Max();
		
		if ( angles[i] < lo )
			cost += parameters_.limit_penalty * ( lo - angles[i] ) * ( lo - angles[i] );
		else if ( angles[i] > hi )
			cost += parameters_.limit_penalty * ( angles[i] - hi ) * ( angles[i] - hi );
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

SphericalSolver::EulerBranch SphericalSolver::GridSearch( const CostFn& f ) const
{
	EulerBranch best_branch;
	
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

SphericalSolver::EulerBranch SphericalSolver::GoldenSearchSection( 
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