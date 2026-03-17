#pragma once

#include "Global.hpp"

namespace SOArm100::Kinematics::Solver
{
struct IKProblem
{
Mat4d target;

VecXd seed;

double position_tolerance;
double rotation_tolerance;

double timeout;
};
}