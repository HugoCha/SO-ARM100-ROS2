#pragma once

#include "Model/JointGroup.hpp"

#include <optional>

namespace SOArm100::Kinematics::Model
{
class JointGroup;
class JointChain;

class BaseAnalyzer
{
public:
[[nodiscard]] static std::optional< JointGroup > Analyze(
	const JointChain& joint_chain,
	const Mat4d& home,
	const std::optional< JointGroup >& wrist_group );

[[nodiscard]] static bool CheckConsistency(
	const JointChain& joint_chain,
	const JointGroup& base_group );
};
}