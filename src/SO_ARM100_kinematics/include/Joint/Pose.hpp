#pragma once

#include "Global.hpp"

#include "Utils/KinematicsUtils.hpp"

namespace SOArm100::Kinematics
{
struct Pose
{
    Vec3d origin{Vec3d::Zero()};
    Vec3d axis{Vec3d::Zero()};

    static Pose FromTransform( const Mat4d& T )
    {
        Pose pose;
        Eigen::AngleAxisd aa( Rotation( T ) );

        pose.axis = aa.axis();
        pose.origin = Translation( T );

        return pose;
    }
};
}