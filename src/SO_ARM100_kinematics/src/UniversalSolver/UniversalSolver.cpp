#include "UniversalSolver/UniversalSolver.hpp"

#include "Global.hpp"
#include "UniversalSolver/UniversalSolution.hpp"
#include "Utils/MathUtils.hpp"
#include <cstdlib>

namespace SOArm100::Kinematics::Solver
{

// ------------------------------------------------------------

UniversalSolver::UniversalSolver( const Model::UniversalModel& model, SolverParameters parameters ) :
    model_( model ),
    parameters_( parameters )
{}

// ------------------------------------------------------------

UniversalSolution UniversalSolver::SolveFromRotation(
	const Mat3d& R_target,
    std::optional< Vec2d > theta_pref ) const
{
    UniversalSolution solution;
    return solution;
}

// ------------------------------------------------------------

UniversalSolution UniversalSolver::SolveFromTwoVectors(
	const Vec3d& p_tcp_local,
	const Vec3d& p_target,
    std::optional< Vec2d > theta_pref ) const
{
	auto joint0 = model_.GetJoint( 0 );
	auto joint1 = model_.GetJoint( 1 );

    const Vec2d prefered = theta_pref.value_or( Vec2d(
        joint0->GetLimits().Center(),
        joint1->GetLimits().Center() ) );

	const Vec3d& b0 = p_tcp_local;
	const Vec3d& b1 = p_target;

	auto solutions = model_.Decompose( b0, b1 );
    auto solution1 = EvaluateSolution( solutions[0], prefered, b0, b1);
    auto solution2 = EvaluateSolution( solutions[1], prefered, b0, b1);

	return solution1.cost < solution2.cost ? solution1 : solution2;
}
    
// ------------------------------------------------------------

UniversalSolution UniversalSolver::EvaluateSolution( 
    const Vec2d& angles,
    const Vec2d& prefered,
	const Vec3d& b0,
	const Vec3d& b1 ) const
{
    UniversalSolution solution;

    auto joint0 = model_.GetJoint( 0 );
	auto joint1 = model_.GetJoint( 1 );

    solution.angles[0] = joint0->GetLimits().Clamp( angles[0] );
    solution.angles[1] = joint0->GetLimits().Clamp( angles[1] );

    auto rotation = AngleAxis( solution.angles[1], joint1->Axis() ) * 
                    AngleAxis( solution.angles[0], joint0->Axis() );
    Vec3d dir = rotation * b0;

    solution.fk_error = Angle( dir, b1 );
    solution.reachable = std::abs( solution.fk_error ) < parameters_.tolerance;
    solution.cost = ComputeCost( solution.fk_error, solution.angles, prefered );

    return solution;
}

// ------------------------------------------------------------

double UniversalSolver::ComputeCost( double fk_error, const Vec2d& angles, const Vec2d& theta_pref ) const
{
    double cost = 0.0;

    cost += FKErrorCost( fk_error );
    cost += DeviationCost( theta_pref, angles );

    return cost;
}

// ------------------------------------------------------------

double UniversalSolver::FKErrorCost( double fk_error ) const
{
    return std::abs( fk_error ) > parameters_.tolerance ? parameters_.fk_error_penalty * std::abs( fk_error ) : 0.0;
}

// ------------------------------------------------------------

double UniversalSolver::DeviationCost( const Vec2d& prefered, const Vec2d& angles ) const
{
    return ( prefered - angles ).squaredNorm();
}

// ------------------------------------------------------------

}