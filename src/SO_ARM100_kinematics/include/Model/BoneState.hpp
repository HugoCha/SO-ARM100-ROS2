#pragma once

#include "Global.hpp"

#include "Model/Bone.hpp"
#include "Model/Pose.hpp"

namespace SOArm100::Kinematics::Model
{
class BoneState
{
public:
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
}