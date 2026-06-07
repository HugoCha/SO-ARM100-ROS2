#pragma once

#include "Global.hpp"

#include <optional>

namespace SOArm100::Kinematics::Model
{
class JointGroup;
class JointChain;
class PlanarNRJointGroup;
class WristJointGroup;

class BaseAnalyzer
{
public:
[[nodiscard]] static std::optional< JointGroup > Analyze(
	const JointChain& joint_chain,
	const Mat4d& home,
	const std::optional< PlanarNRJointGroup >& planar_group,
	const std::optional< WristJointGroup >& wrist_group );

[[nodiscard]] static bool CheckConsistency(
	const JointChain& joint_chain,
	const JointGroup& base_group );
};
}