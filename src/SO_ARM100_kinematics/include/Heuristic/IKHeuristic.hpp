#pragma once

#include "Model/KinematicModel.hpp"

namespace SOArm100::Kinematics
{
namespace Solver
{
class IKProblem;
}

namespace Heuristic
{
class IKHeuristic
{
public:
virtual ~IKHeuristic() = default;

IKHeuristic( KinematicModelConstPtr model ) : model_( model )
{}

virtual Solver::IKProblem Presolve( const Solver::IKProblem& problem ) const = 0;

protected:
KinematicModelConstPtr model_;
};
}
}