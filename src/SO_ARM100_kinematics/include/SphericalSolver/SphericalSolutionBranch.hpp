#pragma once

#include "Global.hpp"

namespace SOArm100::Kinematics::Solver
{
struct SphericalSolutionBranch
{
    double phi;
    Vec3d angles;
    double cost;
};
}