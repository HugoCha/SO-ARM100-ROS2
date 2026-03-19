#pragma once

#include "HybridSolver/BaseNumericWristSolver.hpp"
#include "Model/KinematicModel.hpp"

namespace SOArm100::Kinematics
{
namespace Solver
{
class IKProblem;
}

namespace Heuristic
{
struct IKPresolution;

class IKHeuristic
{
public:
virtual ~IKHeuristic() = default;

IKHeuristic( Model::KinematicModelConstPtr model ) : model_( model )
{}

virtual IKPresolution Presolve( const Solver::IKProblem& problem ) const = 0;

protected:
Model::KinematicModelConstPtr model_;

const JointChain* GetChain() const {
    if ( !model_ ) return nullptr;
    return model_->GetChain(); 
}

const Joint* GetActiveJoint( int index ) const {
    if ( !model_ ) return nullptr;
    return model_->GetChain()->GetActiveJoint( index ).get();
}
};
}
}