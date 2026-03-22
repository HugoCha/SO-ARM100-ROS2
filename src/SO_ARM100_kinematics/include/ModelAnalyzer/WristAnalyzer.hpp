#pragma once

#include "Global.hpp"

#include <optional>

namespace SOArm100::Kinematics::Model
{
class JointChain;
class JointGroup;

class WristAnalyzer
{
public:
static std::optional< JointGroup > Analyze(
	const JointChain& chain,
	const Mat4d& home );
	
static bool CheckConsistency(
	const JointChain& joint_chain,
	const JointGroup& wrist_group );
};
}