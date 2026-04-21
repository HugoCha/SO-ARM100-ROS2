#pragma once

#include "Global.hpp"

namespace SOArm100::Kinematics::Model
{
struct Line3d
{
	Vec3d point;
	Vec3d axis;
};

enum class PointLinePosition
{
	On,
	Off,
};

Line3d Translate( const Line3d& line, const Vec3d& translation );
Line3d Rotate( const Line3d& line, const Quaternion& rotation );
Line3d Transform( const Line3d& line, const Vec3d& translation, const Quaternion& rotation );

Vec3d ProjectPoint( const Line3d& line, const Vec3d& point );
PointLinePosition IsPointOnLine( const Line3d& line, const Vec3d& point );
}