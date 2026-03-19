#include "Model/JointGroup.hpp"

namespace SOArm100::Kinematics::Model
{

// ------------------------------------------------------------

JointGroup CreateFromRange( 
    const std::string& name, 
    int start, 
    int count, 
    const Mat4d& home )
{
    std::vector< int > range_indices( count );
    for ( int i = 0; i < count; i++ )
        range_indices[i] = start + i;
    return { name, range_indices, home };
}

// ------------------------------------------------------------

VecXd JointGroup::GetGroupJoints( const VecXd& full_joints ) const {
    VecXd group_joints( indices.size() );
    for ( int i = 0; i < indices.size(); i++ )
        group_joints[i] = full_joints[indices[i]];
    return group_joints;
}

// ------------------------------------------------------------

void JointGroup::SetGroupJoints( const VecXd& group_joints, VecXd& full_joints ) const
{
    for ( int i = 0; i < indices.size(); i++ )
        full_joints[indices[i]] = group_joints[i];
}

// ------------------------------------------------------------


// ------------------------------------------------------------

}