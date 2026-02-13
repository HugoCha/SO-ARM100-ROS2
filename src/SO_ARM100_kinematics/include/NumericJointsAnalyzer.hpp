#pragma once

#include "BaseJointModel.hpp"
#include "NumericJointsModel.hpp"
#include "Twist.hpp"
#include "WristModel.hpp"

#include <moveit/macros/class_forward.hpp>
#include <moveit/robot_model/joint_model_group.hpp>
#include <optional>
#include <span>

namespace moveit::core
{
MOVEIT_CLASS_FORWARD( JointModel );
}

namespace SOArm100::Kinematics 
{
class NumericJointsAnalyzer
{
public:
[[nodiscard]]static std::optional< NumericJointsModel > Analyze( 
    const std::span< const moveit::core::JointModel* const >& joint_models,
    const std::span< TwistConstPtr > twists,
    const Mat4d& home_configuration, 
    const std::optional< BaseJointModel >& base_joint, 
    const std::optional< WristModel >& wrist_model );
};
}