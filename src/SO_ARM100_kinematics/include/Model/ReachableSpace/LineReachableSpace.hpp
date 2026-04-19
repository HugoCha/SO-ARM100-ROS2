#pragma once

#include "Global.hpp"

#include "Model/Geometry/Line.hpp"
#include "ReachableSpace.hpp"

namespace SOArm100::Kinematics::Model
{
class LineReachableSpace : public ReachableSpace
{
public:
LineReachableSpace( const Vec3d& point, const Vec3d& axis ) :
	line_( point, axis )
{
}

virtual Mat4d GetPossibleReachableTarget( const Mat4d& target ) const override;
virtual bool IsUnreachable( const Mat4d& target ) const override;

private:
Line line_;
};
}