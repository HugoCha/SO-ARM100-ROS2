#pragma once

#include "Model/KinematicModel.hpp"

namespace SOArm100::Kinematics::Solver
{
struct IKProblem;
class IKRunContext;
struct IKSolution;

class IKSolver
{
public:
virtual ~IKSolver() = default;

IKSolver( KinematicModelConstPtr model ) : model_( model )
{}

virtual IKSolution Solve( 
    const IKProblem& problem, 
    const IKRunContext context ) const = 0;

protected:
KinematicModelConstPtr model_;
};
}