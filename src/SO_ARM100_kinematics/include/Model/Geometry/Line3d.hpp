#pragma once

#include "Global.hpp"

namespace SOArm100::Kinematics::Model
{
struct Line3d
{
	Vec3d point;
	Vec3d axis;
};

Line3d Translate( const Line3d& line, const Vec3d& translation );
Line3d Rotate( const Line3d& line, const Quaternion& rotation );
Line3d Transform( const Line3d& line, const Vec3d& translation, const Quaternion& rotation );
}