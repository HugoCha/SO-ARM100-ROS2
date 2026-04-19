#include "Model/Geometry/Base3d.hpp"

#include "Global.hpp"

namespace SOArm100::Kinematics::Model
{

// ------------------------------------------------------------

Base3d Translate( const Base3d& base, const Vec3d& translation )
{
	return Base3d(
		base.x + translation,
		base.y + translation,
		base.z + translation );
}

// ------------------------------------------------------------

Base3d Rotate( const Base3d& base, const Quaternion& rotation )
{
	return Base3d(
		rotation * base.x,
		rotation * base.y,
		rotation * base.z );
}

// ------------------------------------------------------------

Base3d Transform( const Base3d& base, const Vec3d& translation, const Quaternion& rotation )
{
	return Base3d(
		rotation * ( base.x + translation ),
		rotation * ( base.y + translation ),
		rotation * ( base.z + translation ) );
}

// ------------------------------------------------------------

}