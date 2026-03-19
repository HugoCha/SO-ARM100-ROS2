#pragma once

#include "Global.hpp"

#include <memory>

namespace SOArm100::Kinematics
{
class HybridSolverConfiguration;
class IKinematicsSolver;
class JointChain;

class HybridSolverFactory
{
public:
[[nodiscard]] static std::unique_ptr< IKinematicsSolver > Get(
	std::shared_ptr< const JointChain > joint_chain,
	std::shared_ptr< const Mat4d > home_configuration,
	const HybridSolverConfiguration& configuration );
};
}