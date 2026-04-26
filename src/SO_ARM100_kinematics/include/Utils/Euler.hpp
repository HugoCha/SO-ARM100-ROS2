#pragma once

#include "Global.hpp"

#include <optional>

namespace SOArm100::Kinematics 
{
class Euler
{
public:
enum class Configuration
{
    XYZ, XZY, YXZ, YZX, ZXY, ZYX
};

static std::optional< Configuration > ComputeConfiguration( 
    const Vec3d& a1,
    const Vec3d& a2,
    const Vec3d& a3 );

static Vec3d Solve(
    Configuration configuration,
    const Vec3d& a1,
    const Vec3d& a2,
    const Vec3d& a3,
    const Vec3d& old_dir,
    const Vec3d& new_dir );

private:
static Vec3d SolveFromRotation( Configuration configuration, Mat3d rotation );
};
}