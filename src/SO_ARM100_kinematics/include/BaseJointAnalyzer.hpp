#pragma once

#include "BaseJointModel.hpp"
#include "Twist.hpp"
#include "WristModel.hpp"

namespace SOArm100::Kinematics 
{
class BaseJointAnalyzer
{
public:
[[nodiscard]] static std::optional< BaseJointModel > Analyze( 
    TwistConstPtr base_twist, 
    const std::optional< WristModel >& wrist_model );
};
}