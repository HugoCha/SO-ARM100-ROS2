#include "Model/Geometry/Base3d.hpp"

#include "Global.hpp"

namespace SOArm100::Kinematics::Model
{

// ------------------------------------------------------------

Base3d Base3d::Translate( const Vec3d& translation ) const
{
	return Base3d( x_ + translation, y_ + translation, z_ + translation );
}

// ------------------------------------------------------------

Base3d Base3d::Rotate( const Quaternion& rotation ) const
{
	return Base3d( rotation * x_, rotation * y_, rotation * z_ );
}

// ------------------------------------------------------------

Base3d Base3d::Transform(  const Vec3d& translation, const Quaternion& rotation ) const
{
	return Base3d( rotation * ( x_ + translation ), rotation * ( y_ + translation ), rotation * ( z_ + translation ) );
}

// ------------------------------------------------------------

}