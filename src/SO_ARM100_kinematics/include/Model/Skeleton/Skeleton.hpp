#pragma once

#include "Global.hpp"

#include "Articulation.hpp"
#include "Bone.hpp"

#include <memory>
#include <span>

namespace SOArm100::Kinematics::Model
{
class Joint;
class JointGroup;
class KinematicModel;

using KinematicModelConstPtr = std::shared_ptr< const KinematicModel >;

class Skeleton
{
public:
Skeleton( 
    const std::vector< ArticulationConstPtr >& articulations,
    const std::vector< BoneConstPtr >& bones,
    double total_length,
    int joint_count ) :
articulations_( articulations ),
bones_( bones ),
length_( total_length ),
joint_count_( joint_count )
{
}

bool IsEmpty() const {
    return articulations_.empty();
}

int JointCount() const {
	return joint_count_;
}

int ArticulationCount() const {
	return articulations_.size();
}

std::span< const ArticulationConstPtr > Articulations() const {
	return articulations_;
}

ArticulationConstPtr Articulation( int articulation ) const {
	return articulations_[articulation];
}

int BonesCount() const {
	return bones_.size();
}

std::span< const BoneConstPtr > Bones() const {
	return bones_;
}

BoneConstPtr Bone( int bone ) const {
	return bones_[bone];
}

const Vec3d& Center( int articulation ) const {
	return articulations_[articulation]->Center();
}

const Vec3d& Direction( int bone ) const {
	return bones_[bone]->Direction();
}

double Length( int bone ) const {
	return bones_[bone]->Length();
}

double TotalLength() const {
	return length_;
}

private:
int joint_count_;
std::vector< ArticulationConstPtr > articulations_;
std::vector< BoneConstPtr > bones_;
double length_;
};

using SkeletonConstPtr = std::shared_ptr< const Skeleton >;

}