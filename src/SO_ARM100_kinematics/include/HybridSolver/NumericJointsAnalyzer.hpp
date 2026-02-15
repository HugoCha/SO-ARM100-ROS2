#pragma once

#include "Global.hpp"

#include <optional>

namespace SOArm100::Kinematics
{
class BaseJointModel;
class JointChain;
class NumericJointsModel;
class WristModel;

class NumericJointsAnalyzer
{
public:
[[nodiscard]] static std::optional< NumericJointsModel > Analyze(
	const JointChain& joint_chain,
	const Mat4d& home_configuration,
	const std::optional< BaseJointModel >& base_joint,
	const std::optional< WristModel >& wrist_model );
};
}