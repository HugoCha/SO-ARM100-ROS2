#pragma once

#include <ostream>

namespace SOArm100::Kinematics
{
namespace Model
{
class Articulation;
class ArticulationState;
enum class ArticulationType;
class Bone;
class BoneState;
class Joint;
class JointChain;
class JointState;
enum class JointType;
class Limits;
class Link;
struct Pose;
class Skeleton;
class SkeletonState;
class Twist;
}
}

std::ostream& operator << ( std::ostream& os, const SOArm100::Kinematics::Model::ArticulationType& obj );
std::ostream& operator << ( std::ostream& os, const SOArm100::Kinematics::Model::Articulation& obj );
std::ostream& operator << ( std::ostream& os, const SOArm100::Kinematics::Model::ArticulationState& obj );
std::ostream& operator << ( std::ostream& os, const SOArm100::Kinematics::Model::Bone& obj );
std::ostream& operator << ( std::ostream& os, const SOArm100::Kinematics::Model::BoneState& obj );
std::ostream& operator << ( std::ostream& os, const SOArm100::Kinematics::Model::Joint& obj );
std::ostream& operator << ( std::ostream& os, const SOArm100::Kinematics::Model::JointChain& obj );
std::ostream& operator << ( std::ostream& os, const SOArm100::Kinematics::Model::JointState& obj );
std::ostream& operator << ( std::ostream& os, const SOArm100::Kinematics::Model::JointType& obj );
std::ostream& operator << ( std::ostream& os, const SOArm100::Kinematics::Model::Limits& obj );
std::ostream& operator << ( std::ostream& os, const SOArm100::Kinematics::Model::Link& obj );
std::ostream& operator << ( std::ostream& os, const SOArm100::Kinematics::Model::Pose& obj );
std::ostream& operator << ( std::ostream& os, const SOArm100::Kinematics::Model::Skeleton& obj );
std::ostream& operator << ( std::ostream& os, const SOArm100::Kinematics::Model::SkeletonState& obj );
std::ostream& operator << ( std::ostream& os, const SOArm100::Kinematics::Model::Twist& obj );