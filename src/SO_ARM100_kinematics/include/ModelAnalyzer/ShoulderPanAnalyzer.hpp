#pragma once

#include "Model/JointGroup.hpp"

#include <optional>

namespace SOArm100::Kinematics::Model
{
class JointGroup;
class JointChain;

class ShoulderPanAnalyzer
{
public:
[[nodiscard]] static std::optional< JointGroup > Analyze(
	const JointChain& joint_chain,
	const Mat4d& home,
	const std::optional< JointGroup >& wrist_group );
};
}