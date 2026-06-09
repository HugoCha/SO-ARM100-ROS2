#pragma once

#include "Global.hpp"

#include <optional>

namespace SOArm100::Kinematics::Model
{
class FallbackFabrikJointGroup;
class JointGroup;
class JointChain;
class PlanarNRJointGroup;
class WristJointGroup;

class FallbackFabrikAnalyzer
{
public:
[[nodiscard]] static std::optional< FallbackFabrikJointGroup > Analyze(
	const JointChain& joint_chain,
	const Mat4d& home,
	const std::optional< JointGroup >& base_group,
	const std::optional< PlanarNRJointGroup >& planar_group,
	const std::optional< WristJointGroup >& wrist_group );
};
}