#pragma once

#include "Global.hpp"
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

[[nodiscard]] static Vec3d ComputeReferenceDirection(
	const JointChain& chain,
	const Mat4d& home,
	const JointGroup& wrist_group );

[[nodiscard]] static bool CheckConsistency(
	const JointChain& joint_chain,
	const JointGroup& base_group );

private:
[[nodiscard]] static Vec3d ComputeDirection( 
	const JointChain& joint_chain,
	const Mat4d& T_tip );
};
}