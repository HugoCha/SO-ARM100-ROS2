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
class IIKSeedGenerator
{
public:
virtual ~IIKSeedGenerator() = default;

virtual VecXd Generate( const Solver::IKProblem& problem ) const = 0;
};
}
}