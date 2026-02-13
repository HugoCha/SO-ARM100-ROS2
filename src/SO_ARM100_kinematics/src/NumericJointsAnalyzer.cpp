#include "NumericJointsAnalyzer.hpp"

#include "KinematicsUtils.hpp"
#include "NumericJointsModel.hpp"
#include <optional>

namespace SOArm100::Kinematics
{

// ------------------------------------------------------------

const Mat4d ComputeReducedHome( 
    const Mat4d& full_home, 
    const std::optional< WristModel >& wrist_model )
{
    if ( !wrist_model ) return full_home;
    return full_home * Inverse( wrist_model->tcp_in_wrist_at_home );
}

// ------------------------------------------------------------

std::optional< NumericJointsModel > NumericJointsAnalyzer::Analyze( 
    const std::span< const moveit::core::JointModel* const >& joint_models,
    const std::span< TwistConstPtr > twists,
    const Mat4d& home_configuration, 
    const std::optional< BaseJointModel >& base_joint, 
    const std::optional< WristModel >& wrist_model )
{
    int numeric_count = twists.size();
    int numeric_start_index = !base_joint ? 0 : 1;
    int wrist_count = !wrist_model ? 0 : wrist_model->count;

    numeric_count = numeric_count - numeric_start_index - wrist_count;

    if ( numeric_count <= 0 ) return std::nullopt;

    NumericJointsModel numeric_joint_model;
    numeric_joint_model.start_index = numeric_start_index;
    numeric_joint_model.count = numeric_count;

    if ( numeric_count == twists.size() )
    {
        numeric_joint_model.twists = twists;
        numeric_joint_model.home_configuration = home_configuration;
    }
    else
    {
        numeric_joint_model.twists = twists.subspan( numeric_start_index, numeric_count );
        numeric_joint_model.home_configuration = ComputeReducedHome( home_configuration, wrist_model );    
    }

    return numeric_joint_model;
}

// ------------------------------------------------------------

}