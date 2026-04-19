#include "Model/Geometry/Pose.hpp"

#include "Global.hpp"

namespace SOArm100::Kinematics::Model
{

// ------------------------------------------------------------

Pose Translate( const Pose& pose, const Vec3d& translation )
{
	return Pose(
		pose.origin + translation,
		pose.axis );
}

// ------------------------------------------------------------

Pose Rotate( const Pose& pose, const Quaternion& rotation )
{
	return Pose(
		pose.origin,
		rotation * pose.axis );
}

// ------------------------------------------------------------

Pose Transform( const Pose& pose, const Vec3d& translation, const Quaternion& rotation )
{
	return Pose(
		pose.origin + translation,
		rotation * pose.axis );
}

// ------------------------------------------------------------

}