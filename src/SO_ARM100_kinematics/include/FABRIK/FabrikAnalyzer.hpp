#pragma once

#include "Global.hpp"

#include <optional>
#include <string>

namespace SOArm100::Kinematics::Model
{
class JointChain;
struct JointGroup;

constexpr std::string fabrik_group_name = "fabrik";

class FabrikAnalyzer
{
public:
[[nodiscard]] static std::optional< JointGroup > Analyze(
    const JointChain& joint_chain,
    const Mat4d& home,
    int start,
    int count );

[[nodiscard]] static std::optional< JointGroup > Analyze(
    const JointChain& joint_chain,
    const Mat4d& home );

[[nodiscard]] static bool CheckConsistency(
    const JointChain& joint_chain,
    const JointGroup& fabrik_group );
};
}