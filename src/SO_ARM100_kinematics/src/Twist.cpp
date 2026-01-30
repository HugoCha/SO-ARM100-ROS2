#include "Twist.hpp"

#include "KinematicsUtils.hpp"
#include "Types.hpp"

#include <Eigen/Dense>

namespace SOArm100::Kinematics
{
// ------------------------------------------------------------

Twist::Twist( const Vec3d& axis, const Vec3d& point_on_axis )
	: axis_( axis.normalized() )
{
	linear_ = -axis_.cross( point_on_axis );
}

// ------------------------------------------------------------

Twist::Twist( const Vec3d& axis, const Mat4d& transform )
	: axis_( axis.normalized() )
{
	Vec3d point_on_axis = Translation( transform );
	linear_ = -axis_.cross( point_on_axis );
}

// ------------------------------------------------------------

Twist::operator Vec6d () const
{
	Vec6d twist;
	twist << axis_, linear_;
	return twist;
}

// ------------------------------------------------------------

Vec3d Twist::GetAxis() const
{
	return axis_;
}

// ------------------------------------------------------------

Vec3d Twist::GetLinear() const
{
	return linear_;
}

// ------------------------------------------------------------
}
