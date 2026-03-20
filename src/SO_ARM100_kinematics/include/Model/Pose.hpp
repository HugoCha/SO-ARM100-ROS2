#pragma once

#include "Global.hpp"

#include "Utils/KinematicsUtils.hpp"

namespace SOArm100::Kinematics::Model
{
struct Pose
{
	Vec3d origin{ Vec3d::Zero() };
	Vec3d axis{ Vec3d::Zero() };

	static Pose FromTransform( const Mat4d& T, const Vec3d& local_axis )
	{
		Pose pose;

		pose.axis   = Rotation( T ) * local_axis;
		pose.origin = Translation( T );

		return pose;
	}
};
}