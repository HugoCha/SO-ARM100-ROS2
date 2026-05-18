#pragma once

#include <ostream>

namespace SOArm100::Kinematics
{
// Model
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

std::ostream& operator << ( std::ostream& os, const ArticulationType& obj );
std::ostream& operator << ( std::ostream& os, const Articulation& obj );
std::ostream& operator << ( std::ostream& os, const ArticulationState& obj );
std::ostream& operator << ( std::ostream& os, const Bone& obj );
std::ostream& operator << ( std::ostream& os, const BoneState& obj );
std::ostream& operator << ( std::ostream& os, const Joint& obj );
std::ostream& operator << ( std::ostream& os, const JointChain& obj );
std::ostream& operator << ( std::ostream& os, const JointState& obj );
std::ostream& operator << ( std::ostream& os, const JointType& obj );
std::ostream& operator << ( std::ostream& os, const Limits& obj );
std::ostream& operator << ( std::ostream& os, const Link& obj );
std::ostream& operator << ( std::ostream& os, const Pose& obj );
std::ostream& operator << ( std::ostream& os, const Skeleton& obj );
std::ostream& operator << ( std::ostream& os, const SkeletonState& obj );
std::ostream& operator << ( std::ostream& os, const Twist& obj );
}

// Solver
namespace Solver
{
struct IKProblem;
struct IKSolution;
enum class IKSolverState;

std::ostream& operator << ( std::ostream& os, const IKProblem& obj );
std::ostream& operator << ( std::ostream& os, const IKSolution& obj );
std::ostream& operator << ( std::ostream& os, const IKSolverState& obj );
}
}