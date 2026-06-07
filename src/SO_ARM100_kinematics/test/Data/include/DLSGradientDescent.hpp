#pragma once

#include "Global.hpp"
#include "DLS/DLSSolver.hpp"

#include <memory>

namespace SOArm100::Kinematics::Test
{
struct Scenarii
{
	VecXd seed;
	Mat4d target;
};

struct Scenario
{
	std::vector< Scenarii > scenario;
	std::shared_ptr< Solver::DLSSolver > solver;
};

class DLSGradientDescent
{
public:
struct Parameters
{
	int iterations { 100 };
	double noise_percent { 0.0 };
	double h { 1e-5 };
	double learning_rate { 0.1 };
	double gradient_tolerance{ 1e-4 };
};

DLSGradientDescent( int num_scenario );

Solver::DLSSolver::SolverParameters Run( Parameters p ) const;
Solver::DLSSolver::SolverParameters RandomizedSearch( const Solver::DLSSolver::SolverParameters& sp, Parameters p ) const;
double ComputeLoss( const Solver::DLSSolver::SolverParameters& sp ) const;

private:
mutable random_numbers::RandomNumberGenerator rng_;
std::vector< Scenario > robots_scenario_;

void InitializeRobotsScenario( int num_scenario );

VecXd InitializeThetas( const Solver::DLSSolver::SolverParameters& sp, double noise_percent ) const;
Solver::DLSSolver::SolverParameters RandomParameters( const Solver::DLSSolver::SolverParameters& sp, double noise ) const;

double ComputeLoss( const VecXd& theta ) const;
VecXd ComputeGradient( const VecXd& theta, double h ) const;
VecXd GradientDescent( const VecXd& initial, const Parameters& p ) const;

void SetSolverParameters( std::shared_ptr< Solver::DLSSolver > solver, const VecXd& thetas ) const;
Solver::DLSSolver::SolverParameters GetSolverParameters( const VecXd& thetas ) const;
};
}