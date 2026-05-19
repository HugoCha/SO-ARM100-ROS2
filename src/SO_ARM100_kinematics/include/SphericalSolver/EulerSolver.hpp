#pragma once

#include "Global.hpp"

#include "EulerModel.hpp"

#include <functional>

namespace SOArm100::Kinematics::Solver
{
struct SphericalSolution;
struct SphericalSolutionBranch;

class EulerSolver
{
public:
struct SolverParameters
{
	int phi_samples { 360 };   ///< coarse grid resolution over [0,2π)
	int refine_max_iters { 200 };   ///< golden-section refinement iterations
	double reachability_tol { 1e-4 };  ///< sphere radius mismatch tolerance [m]
	double singularity_tol { 0.05 };  ///< cos(θ₂) threshold for gimbal lock
	double limit_penalty { 1e6 };   ///< quadratic penalty weight for limits
	double singularity_penalty { 1e4 };  ///< penalty weight for singular configs
};

explicit EulerSolver( const Model::EulerModel& model, SolverParameters parameters );

const Model::EulerModel& GetEulerModel()  const {
	return model_;
}

const SolverParameters& GetParameters() const {
	return parameters_;
}

SolverParameters& GetParameters(){
	return parameters_;
}
	
[[nodiscard]] SphericalSolution SolveFromRotation(
	const Mat3d& R_target ) const;
	
// Returns first found solution
[[nodiscard]] SphericalSolution SolveFromTwoVectors( 	
	const Vec3d& p_tcp_local,
	const Vec3d& p_target ) const;

// Returns optimized solution
[[nodiscard]] SphericalSolution SolveAndOptimizeFromTwoVectors(
	const Vec3d& p_tcp_local,
	const Vec3d& p_target,
	std::optional< Vec3d > theta_pref = std::nullopt ) const;

private:
using CostFn = std::function< SphericalSolutionBranch ( double phi ) >;

Model::EulerModel model_;
SolverParameters parameters_;

double DeviationCost( const Vec3d& prefered, const Vec3d& angles ) const;
double LimitViolationCost( const Vec3d& angles, double violation_weight ) const;
double SingularityCost( const Mat3d& R_canonical ) const;

SphericalSolutionBranch GridSearch( const CostFn& f ) const;
SphericalSolutionBranch FirstSolutionSearch( const CostFn& f ) const;

SphericalSolutionBranch GoldenSearchSection(
	const CostFn& f,
	double a,
	double b,
	double tol = 1e-9 ) const;

bool CheckLimits( Vec3d& angles ) const;
};
}