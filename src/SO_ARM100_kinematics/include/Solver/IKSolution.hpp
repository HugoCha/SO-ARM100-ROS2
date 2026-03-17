#pragma once

#include "Global.hpp"

#include "IKSolverState.hpp"

namespace SOArm100::Kinematics::Solver
{
struct IKSolution
{
VecXd joints;

double position_error = std::numeric_limits<double>::infinity();
double orientation_error = std::numeric_limits<double>::infinity();

double score = std::numeric_limits<double>::infinity();

int iterations = 0;

IKSolverState state;

[[nodiscard]] bool Success() const {
    return state == IKSolverState::Converged;
}
};
}