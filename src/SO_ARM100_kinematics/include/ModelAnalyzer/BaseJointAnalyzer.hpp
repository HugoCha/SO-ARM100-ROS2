#pragma once

#include "Model/JointGroup.hpp"
#include <optional>

namespace SOArm100::Kinematics::Model
{
class JointGroup;
class JointChain;

class BaseJointAnalyzer
{
public:
[[nodiscard]] static std::optional< Model::JointGroup > Analyze(
	const JointChain& joint_chain,
	const std::optional< WristModel >& wrist_model );
};
}