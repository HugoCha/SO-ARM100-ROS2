#pragma once

#include "Global.hpp"

namespace SOArm100::Kinematics
{
class JointChain;
class WristModel;

class WristAnalyzer
{
public:
static std::optional< WristModel > Analyze(
	const JointChain& joint_chain,
	const Mat4d& home_configuration );
};
}