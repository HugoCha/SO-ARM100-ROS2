#include "Utils/MathUtils.hpp"

#include "Global.hpp"

namespace SOArm100::Kinematics
{

// ------------------------------------------------------------

Vec3d ProjectPointOnPlane(
	const Vec3d& point,
	const Vec3d& plane_point,
	const Vec3d& plane_normal )
{
	Vec3d n = plane_normal.normalized();
	Vec3d dir = point - plane_point;
	Vec3d project = dir - dir.dot( n ) * n;

	if ( project.norm() < epsilon )
		return plane_point;

	return plane_point + project;
}

// ------------------------------------------------------------

Vec3d ProjectVectorOnPlane(
	const Vec3d& vector,
	const Vec3d& plane_normal )
{
	Vec3d n = plane_normal.normalized();
	Vec3d project = vector - vector.dot( n ) * n;

	if ( project.norm() < epsilon )
		return Vec3d::Zero();

	return project;
}

// ------------------------------------------------------------

Vec3d ProjectPointOnAxis(
	const Vec3d& point,
	const Vec3d& origin,
	const Vec3d& axis )
{
	Vec3d n = axis.normalized();
	Vec3d dir = point - origin;
	Vec3d project = dir.dot( n ) * n;

	if ( project.norm() < epsilon )
		return origin;

	return origin + project;
}

// ------------------------------------------------------------

double Angle( const Vec3d& V1, const Vec3d& V2 )
{
	Vec3d V1xV2 = V1.cross( V2 );
	double V1dotV2 = V1.dot( V2 );

	double angle = 0;

	if ( V1xV2.norm() < epsilon )
	{
		if ( V1dotV2 > 0 )
			return 0.0;
		return M_PI;
	}

	double sin_angle = V1xV2.norm();
	double cos_angle = V1dotV2;
	return std::atan2( sin_angle, cos_angle );
}

// ------------------------------------------------------------

double SignedAngle( const Vec3d& V1, const Vec3d& V2, const Vec3d& normal )
{
	double V1_norm = V1.norm();
	double V2_norm = V2.norm();
	double n_norm  = normal.norm();

	if ( V1_norm < epsilon || V2_norm < epsilon || n_norm < epsilon )
		return 0.0;

	Vec3d V1_normalize = V1 / V1_norm;
	Vec3d V2_normalize = V2 / V2_norm;
	Vec3d n_normalize = normal / n_norm;

	Vec3d V1xV2 = V1_normalize.cross( V2_normalize );
	double V1dotV2 = V1_normalize.dot( V2_normalize );

	double sin_angle = V1xV2.dot( n_normalize );
	double cos_angle = std::clamp( V1dotV2, -1.0, 1.0 );

	return std::atan2( sin_angle, cos_angle );
}

// ------------------------------------------------------------

double AngleAroundAxis( const Vec3d& V1, const Vec3d& V2, const Vec3d& axis )
{
	double V1_norm = V1.norm();
	double V2_norm = V2.norm();
	double axis_norm  = axis.norm();

	if ( V1_norm < epsilon || V2_norm < epsilon || axis_norm < epsilon )
		return 0.0;

	Vec3d V1_normalize = V1 / V1_norm;
	Vec3d V2_normalize = V2 / V2_norm;
	Vec3d axis_normalize = axis / axis_norm;

	Vec3d V1xV2 = V1_normalize.cross( V2_normalize );
	double V1dotV2 = V1_normalize.dot( V2_normalize );
	double axisdotV1 = axis_normalize.dot( V1_normalize );
	double axisdotV2 = axis_normalize.dot( V2_normalize );

	if ( std::abs( 1.0 - std::abs( axisdotV1 ) ) < epsilon ||
	     std::abs( 1.0 - std::abs( axisdotV2 ) ) < epsilon )
		return 0.0;

	double sin_angle = axis_normalize.dot( V1xV2 );
	double cos_angle = std::clamp( V1dotV2 - axisdotV1 * axisdotV2, -1.0, 1.0 );

	return std::atan2( sin_angle, cos_angle );
}

// ------------------------------------------------------------

double OrthogonalityError( const Mat3d& Q )
{
	return ( Q.transpose() * Q - Mat3d::Identity() ).norm();
}

// ------------------------------------------------------------

}