/*
#include "DLSGradientDescent.hpp"

#include "Global.hpp"

#include "DLS/DLSSolver.hpp"
#include "RobotModelTestData.hpp"
#include "Solver/IKProblem.hpp"
#include "Solver/IKRunContext.hpp"
#include "Solver/IKSolution.hpp"

#include <limits>
#include <memory>

namespace SOArm100::Kinematics::Test
{

// ------------------------------------------------------------

DLSGradientDescent::DLSGradientDescent( int num_scenario )
{
	InitializeRobotsScenario( num_scenario );
}

// ------------------------------------------------------------

void DLSGradientDescent::InitializeRobotsScenario( int num_scenario )
{
	std::vector< Scenario > robots_scenario;
	for ( const auto& robotPair : Data::GetAllRobots() )
	{
		auto& model = robotPair.second;
		Scenario s;
		s.solver = std::make_shared< Solver::DLSSolver >( model );

		for ( int i = 0; i < num_scenario; i++ )
		{
			VecXd joints = model->GetChain()->RandomValidJoints( rng_ );
			VecXd seed = model->GetChain()->RandomValidJointsNear( rng_, joints, 0.3 );
			Mat4d target;
			model->ComputeFK( joints, target );
			s.scenario.emplace_back( Scenarii{ seed, target } );
		}

		robots_scenario.emplace_back( s );
	}

	robots_scenario_ = robots_scenario;
}

// ------------------------------------------------------------

VecXd DLSGradientDescent::InitializeThetas(
	const Solver::DLSSolver::SolverParameters& p,
	double noise_percent ) const
{
	VecXd thetas( 11 );

	auto add_noise_double = [&]( double value ) -> double {
								double min = value - noise_percent * value / 2.0;
								double max = value + noise_percent * value / 2.0;
								return rng_.uniformReal( min, max );
							};

	auto add_noise_int = [&]( int value ) -> double {
							 int min = value - noise_percent * value / 2.0;
							 int max = value + noise_percent * value / 2.0;
							 return rng_.uniformInteger( min, max );
						 };

	thetas[0] = add_noise_int( p.max_iterations );
	thetas[1] = add_noise_double( p.min_step );
	thetas[2] = thetas[1] + add_noise_double( p.max_step - p.min_step );
	thetas[3] = add_noise_double( p.line_search_factor );
	thetas[4] = add_noise_double( p.min_damping );
	thetas[5] = thetas[4] + add_noise_double( p.max_damping - p.min_damping );
	thetas[6] = add_noise_double( p.max_dq );
	thetas[7] = add_noise_double( p.min_sv_tolerance );
	thetas[8] = add_noise_int( p.max_stalle_iterations );
	thetas[9] = add_noise_double( p.translation_weight );
	thetas[10] = add_noise_double( p.rotation_weight );

	return thetas;
}

// ------------------------------------------------------------

Solver::DLSSolver::SolverParameters DLSGradientDescent::GetSolverParameters( const VecXd& thetas ) const
{
	Solver::DLSSolver::SolverParameters p;

	p.max_iterations        = std::clamp( ( int )thetas[0], 10, 200 );
	p.min_step              = std::clamp( thetas[1], 0.001, 0.5 );
	p.max_step              = std::clamp( thetas[1] + thetas[2], 0.05, 1.0 );
	p.line_search_factor    = std::clamp( thetas[3], 0.1, 0.9 );
	p.min_damping           = std::clamp( thetas[4], 0.001, 0.5 );
	p.max_damping           = std::clamp( thetas[4] + thetas[5], 0.05, 1.0 );
	p.max_dq                = std::clamp( thetas[6], 0.1, 0.9 );
	p.min_sv_tolerance      = std::clamp( thetas[7], 1e-6, 1e-2 );
	p.max_stalle_iterations = std::clamp( ( int )thetas[8], 1, 29 );
	p.translation_weight    = std::clamp( thetas[9], 1.0, 20.0 );
	p.rotation_weight       = std::clamp( thetas[10], 1.0, 20.0 );

	return p;
}

// ------------------------------------------------------------

void DLSGradientDescent::SetSolverParameters(
	std::shared_ptr< Solver::DLSSolver > solver,
	const VecXd& thetas ) const
{
	Solver::DLSSolver::SolverParameters p;

	p.max_iterations        = thetas[0];
	p.min_step              = thetas[1];
	p.max_step              = thetas[1] + thetas[2];
	p.line_search_factor    = thetas[3];
	p.min_damping           = thetas[4];
	p.max_damping           = thetas[4] + thetas[5];
	p.max_dq                = thetas[6];
	p.min_sv_tolerance      = thetas[7];
	p.max_stalle_iterations = thetas[8];
	p.translation_weight    = thetas[9];
	p.rotation_weight       = thetas[10];

	solver->SetParameters( p );
}

// ------------------------------------------------------------

double DLSGradientDescent::ComputeLoss( const VecXd& theta ) const
{
	Solver::DLSSolver::SolverParameters p = GetSolverParameters( theta );
	return ComputeLoss( p );
}

// ------------------------------------------------------------

VecXd DLSGradientDescent::ComputeGradient( const VecXd& theta, double h ) const
{
	VecXd grad = VecXd::Zero( theta.size() );
	VecXd ones = VecXd::Ones( theta.size() );
	for ( int i = 0; i < theta.size(); i++ )
	{
		VecXd theta_inf = theta;
		VecXd theta_sup = theta;
		theta_inf[i] = theta[i] - h;
		theta_sup[i] = theta[i] + h;
		grad[i] = ( ComputeLoss( theta_sup ) - ComputeLoss( theta_inf ) ) / ( 2 * h );
	}
	return grad;
}

// ------------------------------------------------------------

Solver::DLSSolver::SolverParameters DLSGradientDescent::RandomParameters(
	const Solver::DLSSolver::SolverParameters& p,
	double noise ) const
{
	VecXd theta = InitializeThetas( p, noise );
	return GetSolverParameters( theta );
}

// ------------------------------------------------------------

Solver::DLSSolver::SolverParameters DLSGradientDescent::RandomizedSearch(
	const Solver::DLSSolver::SolverParameters& sp,
	Parameters p ) const
{
	VecXd best_theta;
	double best_loss = std::numeric_limits< double >::infinity();

	for ( int i = 0; i < p.iterations; i++ )
	{
		VecXd random = InitializeThetas( sp, p.noise_percent );
		double loss = ComputeLoss( random );

		if ( loss < best_loss )
		{
			best_loss = loss;
			best_theta = random;
		}
	}

	return GetSolverParameters( best_theta );
}

// ------------------------------------------------------------

VecXd DLSGradientDescent::GradientDescent( const VecXd& initial, const Parameters& p ) const
{
	VecXd theta = initial;
	VecXd grad = VecXd::Zero( theta.size() );
	for ( int i = 0; i < p.iterations; i++ )
	{
		grad = ComputeGradient( theta, p.h );
		for ( int i = 0; i < theta.size(); i++ )
			theta[i] = std::max( 1e-4, theta[i] + p.learning_rate * grad[i] );

		if ( grad.squaredNorm() < p.gradient_tolerance )
			return theta;
	}
	return theta;
}

// ------------------------------------------------------------

Solver::DLSSolver::SolverParameters DLSGradientDescent::Run( Parameters p ) const
{
	Solver::DLSSolver::SolverParameters initial_sp;
	VecXd initial_p = InitializeThetas( initial_sp, std::clamp( p.noise_percent, 0.0, 1.0 ) );
	VecXd final_p = GradientDescent( initial_p, p );
	return GetSolverParameters( final_p );
}

// ------------------------------------------------------------

double DLSGradientDescent::ComputeLoss( const Solver::DLSSolver::SolverParameters& p ) const
{
	Solver::IKRunContext context;
	double loss = 0;

	auto cost = [&]( const Solver::IKProblem& problem, const Solver::IKSolution& solution ){
					return solution.error / error_tolerance +
					       solution.iterations / ( double )p.max_iterations +
					       ( solution.Success() ? 0.0 : 10.0 );
				};

	VecXd theta = InitializeThetas( p, 0.0 );

	for ( const auto& robot_scenarii : robots_scenario_ )
	{
		Solver::IKProblem problem;
		Solver::IKSolution solution;
		SetSolverParameters( robot_scenarii.solver, theta );
		double robot_loss = 0;
		for ( const auto& s : robot_scenarii.scenario )
		{
			problem.seed = s.seed;
			problem.target = s.target;

			solution = robot_scenarii.solver->Solve( problem, context );
			robot_loss += cost( problem, solution );
		}

		robot_loss /= ( double )robot_scenarii.scenario.size();
		loss += robot_loss / ( double )robots_scenario_.size();
	}

	return loss;
}

// ------------------------------------------------------------

}
*/