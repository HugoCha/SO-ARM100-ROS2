#include "Twist.hpp"

namespace SOArm100::Kinematics
{
// ------------------------------------------------------------

Twist::Twist(Eigen::Vector3d axis, Eigen::Vector3d point_on_axis)
: axis_(axis.normalized() )
{
  linear_ = -axis_.cross(point_on_axis);
}

// ------------------------------------------------------------

Twist::Twist(Eigen::Vector3d axis, Eigen::Matrix4d transform)
: axis_(axis.normalized() )
{
  Eigen::Vector3d point_on_axis = transform.block<3, 1>(0, 3);
  linear_ = -axis_.cross(point_on_axis);
}

// ------------------------------------------------------------

Twist::Twist(const Twist & other)
: axis_(other.axis_), linear_(other.linear_)
{
}

// ------------------------------------------------------------

Twist::~Twist()
{
}

// ------------------------------------------------------------

Eigen::Vector3d Twist::GetAxis() const
{
  return axis_;
}

// ------------------------------------------------------------

Eigen::Vector3d Twist::GetLinear() const
{
  return linear_;
}

// ------------------------------------------------------------
}
