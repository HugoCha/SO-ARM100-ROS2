#pragma once

#include "Global.hpp"

#include "EulerModel.hpp"

#include <functional>

namespace SOArm100::Kinematics::Solver
{
class SphericalSolver
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

struct IKResult {
    Vec3d  angles;         ///< physical joint angles (θ₁, θ₂, θ₃) [rad]
    double phi;            ///< optimal free angle φ [rad]
    double cost;           ///< optimizer cost at solution
    double fk_error;       ///< ‖FK(θ)·p_tcp_local − p_target‖ [m]
    double singularity_margin; ///< 0=singular, 1=far (cos θ₂ in canonical frame)
    bool   reachable;      ///< all joints within limits?
    bool   near_singular;  ///< middle joint near gimbal lock?
};

explicit SphericalSolver( const Model::EulerModel& model, SolverParameters parameters );

const Model::EulerModel& GetEulerModel()  const { 
    return model_; 
}

const SolverParameters& GetParameters() const { 
    return parameters_;   
}

SolverParameters& GetParameters() { 
    return parameters_;   
}

[[nodiscard]] IKResult Solve( 
    const Vec3d& p_tcp_local,
    const Vec3d& p_target,
    std::optional<Vec3d> theta_pref = std::nullopt) const;

private:
struct EulerBranch
{
    double phi;
    Vec3d angles;
    double cost;
};

using CostFn = std::function< EulerBranch( double phi ) >;

Model::EulerModel model_;    
SolverParameters parameters_;   

double DeviationCost( const Vec3d& prefered, const Vec3d& angles ) const;
double LimitViolationCost( const Vec3d& angles ) const;
double SingularityCost( const Mat3d& R_canonical ) const;

EulerBranch GridSearch( const CostFn& f ) const;

EulerBranch GoldenSearchSection( 
    const CostFn& f,
    double a, 
    double b,
    double tol = 1e-9 ) const;

bool CheckLimits( Vec3d& angles ) const;


Mat3d ComputeRMinimal(
    const Vec3d& old_dir,
    const Vec3d& new_dir ) const;
};
}