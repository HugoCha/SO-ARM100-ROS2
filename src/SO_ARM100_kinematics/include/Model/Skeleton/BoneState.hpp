#pragma once

#include "Global.hpp"

#include "Bone.hpp"
#include "Model/Geometry/Pose.hpp"
#include <memory>

namespace SOArm100::Kinematics::Model
{
class BoneState
{
public:
BoneState( const Vec3d& origin, const Vec3d& direction ) :
	bone_( nullptr ),
	pose_( origin, direction )
{}

BoneState( BoneConstPtr bone ) :
	bone_( bone ),
	pose_( bone->Origin(), bone->Direction() )
{
}

const Vec3d& Origin() const {
	return pose_.origin;
}

Vec3d& Origin(){
	return pose_.origin;
}

const Vec3d& Direction() const {
	return pose_.axis;
}

Vec3d& Direction(){
	return pose_.axis;
}

BoneConstPtr GetBone() const {
	return bone_;
}

private:
BoneConstPtr bone_;
Pose pose_;
};

using BoneStatePtr = std::shared_ptr< BoneState >;

}