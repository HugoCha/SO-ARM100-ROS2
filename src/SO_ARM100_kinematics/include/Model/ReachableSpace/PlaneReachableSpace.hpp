#pragma once

#include "Global.hpp"

#include "Model/Geometry/Plane.hpp"
#include "ReachableSpace.hpp"

namespace SOArm100::Kinematics::Model
{
class PlaneReachableSpace : public ReachableSpace
{
public:
PlaneReachableSpace( const Vec3d& point, const Vec3d& normal ) :
	plane_( point, normal )
{
}

virtual Mat4d GetPossibleReachableTarget( const Mat4d& target ) const override;
virtual bool IsUnreachable( const Mat4d& target ) const override;

private:
Plane plane_;
};
}