#pragma once

#include "Global.hpp"

namespace SOArm100::Kinematics
{
namespace Solver
{
struct IKProblem;
}

namespace Seed
{
class IKSeedGenerator
{
    public:
    virtual ~IKSeedGenerator() = default;
    
    virtual VecXd Generate( const Solver::IKProblem& problem ) const = 0;
};
}
}