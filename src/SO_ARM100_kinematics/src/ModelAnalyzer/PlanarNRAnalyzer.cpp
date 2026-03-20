#include "ModelAnalyzer/PlanarNRAnalyzer.hpp"

#include "Global.hpp"
#include "Model/JointChain.hpp"
#include "Model/JointGroup.hpp"
#include <vector>

namespace SOArm100::Kinematics::Model
{

// ------------------------------------------------------------

Vec3d GetJointAxis( JointChain chain, int index );
Mat4d GetJointOrigin( JointChain chain, int index );

// ------------------------------------------------------------

std::vector< JointGroup > PlanarNRAnalyzer::Analyze( 
    const JointChain& chain, 
    const Mat4d& home,
    int start_idx,
    int count )
{
    const int n_joints = chain.GetActiveJointCount();

    if ( start_idx < 0 || count <= 1 || start_idx + count > n_joints )
        return {};

    std::vector< JointGroup > planar_groups;
    auto active_joints = chain.GetActiveJoints();

    int sub_planar_start = start_idx;
    int sub_planar_count = 1;
    auto axis = GetJointAxis( chain, start_idx );

    while ( sub_planar_start < count )
    {
        if ( sub_planar_start + sub_planar_count < count &&
             axis == GetJointAxis( chain, sub_planar_start + sub_planar_count ) )
        {
            sub_planar_count++;
        }
        else
        {
            if ( sub_planar_count >= 2 )
            {
                std::string name = "planar_" + std::to_string( planar_groups.size() );
                JointGroupType type = sub_planar_count == 2 ? 
                    JointGroupType::Planar2R : JointGroupType::PlanarNR;
                
                Mat4d planar_group_home;
                if ( start_idx + count >= n_joints )
                {
                    planar_group_home = home;
                }
                else
                {
                    planar_group_home = GetJointOrigin( chain, sub_planar_start + sub_planar_count );
                }
                
                auto sub_planar_group = JointGroup::CreateFromRange(
                    name,
                    type, 
                    sub_planar_start, 
                    sub_planar_count -  1, 
                    planar_group_home );

                planar_groups.emplace_back( sub_planar_group );
            }

            sub_planar_start += sub_planar_count;
            sub_planar_count = 1;
            if ( sub_planar_start < n_joints )
                axis = GetJointAxis( chain, sub_planar_start );
        }
    }

    return planar_groups;
}

// ------------------------------------------------------------

std::vector< JointGroup > PlanarNRAnalyzer::Analyze( 
    const JointChain& chain, 
    const Mat4d& home )
{
    return Analyze( chain, home, 0, chain.GetActiveJointCount() );
}

// ------------------------------------------------------------

Vec3d GetJointAxis( JointChain chain, int index )
{
    auto joint = chain.GetActiveJoint( index );
    if ( !joint ) return Vec3d::Zero();
    return joint->Axis();
}

// ------------------------------------------------------------

Mat4d GetJointOrigin( JointChain chain, int index )
{
    auto joint = chain.GetActiveJoint( index );
    if ( !joint ) return Mat4d::Identity();
    return joint->GetLink().GetJointOrigin();
}

// ------------------------------------------------------------

}