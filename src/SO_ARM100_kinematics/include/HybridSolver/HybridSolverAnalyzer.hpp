#pragma once

#include "Global.hpp"

namespace SOArm100::Kinematics
{
class HybridSolverConfiguration;
class JointChain;

class HybridSolverAnalyzer
{
public:
[[nodiscard]] static const HybridSolverConfiguration AnalyzeConfiguration(
	const JointChain& joint_chain,
	const Mat4d& home_configuration );
};
}