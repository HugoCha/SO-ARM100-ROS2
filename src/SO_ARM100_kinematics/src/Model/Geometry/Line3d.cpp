#include "Model/Geometry/Line3d.hpp"

#include "Global.hpp"

namespace SOArm100::Kinematics::Model
{

// ------------------------------------------------------------

Line3d Translate( const Line3d& line, const Vec3d& translation )
{
	return Line3d(
		line.point + translation,
		line.axis );
}

// ------------------------------------------------------------

Line3d Rotate( const Line3d& line, const Quaternion& rotation )
{
	return Line3d(
		line.point,
		rotation * line.axis );
}

// ------------------------------------------------------------

Line3d Transform( const Line3d& line, const Vec3d& translation, const Quaternion& rotation )
{
	return Line3d(
		line.point + translation,
		rotation * line.axis );
}

// ------------------------------------------------------------

Vec3d ProjectPoint( const Line3d& line, const Vec3d& point )
{
	Vec3d dir = point - line.point;
	return line.axis.dot( dir ) * line.axis;
}

// ------------------------------------------------------------

PointLinePosition IsPointOnLine( const Line3d& line, const Vec3d& point )
{
	Vec3d dir = point - line.point;

	if ( line.axis.cross( dir ).norm() < epsilon )
		return PointLinePosition::On;

	return PointLinePosition::Off;
}

// ------------------------------------------------------------

}