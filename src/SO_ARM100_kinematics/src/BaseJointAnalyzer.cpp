#include "Global.hpp"

#include "BaseJointAnalyzer.hpp"
#include <optional>

namespace SOArm100::Kinematics 
{

// ------------------------------------------------------------

std::optional< BaseJointModel > BaseJointAnalyzer::Analyze( 
    TwistConstPtr base_twist, 
    const std::optional< WristModel >& wrist_model ) 
{
    if ( !wrist_model || !base_twist || !base_twist->IsRevolute() ) return std::nullopt;
    
    const Vec3d wrist_center_at_home = wrist_model->center_at_home;
    const Vec3d omega = base_twist->GetAxis();
    const Vec3d r = wrist_center_at_home - base_twist->GetLinear();
    Vec3d reference_direction = r - r.dot( omega ) * omega;

    return BaseJointModel{ reference_direction, base_twist };
}

// ------------------------------------------------------------

}