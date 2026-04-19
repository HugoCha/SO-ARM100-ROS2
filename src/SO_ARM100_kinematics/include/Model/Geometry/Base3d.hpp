#pragma once

#include "Global.hpp"

namespace SOArm100::Kinematics::Model
{
class Base3d
{
public:
Base3d( const Vec3d& x, const Vec3d& y, const Vec3d& z ) :
	x_( x ), y_( y ), z_( z )
{
}

const Vec3d& X() const {
	return x_;
}
const Vec3d& Y() const {
	return y_;
}
const Vec3d& Z() const {
	return z_;
}

Base3d Translate( const Vec3d& translation ) const;
Base3d Rotate( const Quaternion& rotation ) const;
Base3d Transform( const Vec3d& translation, const Quaternion& rotation ) const;

private:
Vec3d x_;
Vec3d y_;
Vec3d z_;
};
}