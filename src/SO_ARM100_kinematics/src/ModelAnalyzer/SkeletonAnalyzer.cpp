#include "ModelAnalyzer/SkeletonAnalyzer.hpp"

#include "Model/Articulation.hpp"
#include "Model/Skeleton.hpp"
#include "Model/Joint.hpp"

namespace SOArm100::Kinematics::Model
{

// ------------------------------------------------------------

SkeletonConstPtr SkeletonAnalyzer::Analyze(
	const std::vector< JointConstPtr >& joints,
	const Mat4d& tip )
{
	return nullptr;
}

// ------------------------------------------------------------

}