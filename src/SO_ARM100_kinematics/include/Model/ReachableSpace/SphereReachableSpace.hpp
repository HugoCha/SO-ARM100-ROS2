#pragma once

#include "Global.hpp"

#include "Model/Geometry/Sphere3d.hpp"
#include "ReachableSpace.hpp"

namespace SOArm100::Kinematics::Model
{
class SphereReachableSpace : public ReachableSpace
{
public:
SphereReachableSpace( const Vec3d& origin, double radius, double margin_percent = 0.01 ) :
	sphere_( origin, radius ),
	margin_percent_( margin_percent )
{
}

virtual Mat4d GetPossibleReachableTarget( const Mat4d& target ) const override;
virtual bool IsUnreachable( const Mat4d& target ) const override;

private:
const double margin_percent_;
Sphere3d sphere_;
};
}