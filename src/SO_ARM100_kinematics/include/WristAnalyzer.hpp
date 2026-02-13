#pragma once

#include "Global.hpp"

#include "Twist.hpp"
#include "WristModel.hpp"

#include <moveit/macros/class_forward.hpp>
#include <span>

namespace moveit::core
{
MOVEIT_CLASS_FORWARD( JointModel );
}

namespace SOArm100::Kinematics
{
class WristAnalyzer
{
public:
static std::optional< WristModel > Analyze( 
    const std::span< const moveit::core::JointModel* const >& joint_models,
    const std::span< TwistConstPtr >& twists, 
    const Mat4d& home_configuration );
};
}