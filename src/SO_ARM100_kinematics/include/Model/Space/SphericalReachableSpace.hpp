#pragma once

#include "Global.hpp"

#include "ReachableSpace.hpp"

namespace SOArm100::Kinematics::Model
{
class SphericalReachableSpace : public ReachableSpace
{
public:
SphericalReachableSpace( const Vec3d& origin, double radius ) :
    origin_( origin ),
    radius_( radius )
{}

virtual Mat4d GetPossibleReachableTarget( const Mat4d& target ) const override;
virtual bool IsUnreachable( const Mat4d& target ) const override;

private:
Vec3d origin_;
double radius_;
};
}