#pragma once

#include <optional>

namespace SOArm100::Kinematics
{
class BaseJointModel;
class JointChain;
class WristModel;

class BaseJointAnalyzer
{
public:
[[nodiscard]] static std::optional< BaseJointModel > Analyze(
	const JointChain& joint_chain,
	const std::optional< WristModel >& wrist_model );
};
}