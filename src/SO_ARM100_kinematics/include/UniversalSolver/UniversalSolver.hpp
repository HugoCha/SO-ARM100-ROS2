#pragma once

#include "Global.hpp"

#include "UniversalModel.hpp"

namespace SOArm100::Kinematics::Solver
{
struct UniversalSolution;

class UniversalSolver
{
public:
struct SolverParameters
{
double tolerance { 5e-2 };
double fk_error_penalty { 1e6 };
};
explicit UniversalSolver( const Model::UniversalModel& model, SolverParameters parameters );

const Model::UniversalModel& GetModel() const {
    return model_;
}

[[nodiscard]] UniversalSolution SolveFromRotation(
	const Mat3d& R_target,
    std::optional< Vec2d > theta_pref = std::nullopt ) const;

[[nodiscard]] UniversalSolution SolveFromTwoVectors(
    const Vec3d& p_tcp_local,
    const Vec3d& p_target,
    std::optional< Vec2d > theta_pref = std::nullopt ) const;

private:
SolverParameters parameters_;
Model::UniversalModel model_;

double DeviationCost( const Vec2d& prefered, const Vec2d& angles ) const;
double SingularityCost( const Mat3d& R_canonical ) const;
double FKErrorCost( double fk_error ) const;

UniversalSolution EvaluateSolution( 
    const Vec2d& angles,
    const Vec2d& prefered,
	const Vec3d& b0,
	const Vec3d& b1 ) const;

double ComputeCost( double fk_error, const Vec2d& angles, const Vec2d& theta_pref ) const;
};
}