#pragma once

#include "Global.hpp"

#include <memory>
#include <vector>

namespace SOArm100::Kinematics::Model
{
class Joint;
class Skeleton;

using JointConstPtr = std::shared_ptr< const Joint >;
using SkeletonConstPtr = std::shared_ptr< const Skeleton >;

class SkeletonAnalyzer
{
public:
static SkeletonConstPtr Analyze(
	const std::vector< JointConstPtr >& joints,
	const Mat4d& tip );
};
}