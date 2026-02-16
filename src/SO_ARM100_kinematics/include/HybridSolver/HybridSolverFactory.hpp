#pragma once

#include <memory>

namespace SOArm100::Kinematics
{
class HybridSolverConfiguration;
class IKinematicsSolver;
class JointChain;

class HybridSolverFactory
{
public:
[[nodiscard]] static std::unique_ptr< IKinematicsSolver > Get( JointChain joint_chain, const HybridSolverConfiguration& configuration );
};
}