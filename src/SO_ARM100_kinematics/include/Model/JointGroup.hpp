#pragma once

#include "Global.hpp"
#include <string>
#include <vector>

namespace SOArm100::Kinematics::Model
{
struct JointGroup
{
    std::string name;
    std::vector<int> indices;
    Mat4d tip_home;

    VecXd GetJoints( const VecXd& full_joints ) const {
        VecXd group_joints( indices.size() );
        for ( int i = 0; i < indices.size(); i++ )
            group_joints[i] = full_joints[indices[i]];
        return group_joints;
    }

    void SetJoints( const VecXd& group_joints, VecXd& full_joints ) const
    {
        for ( int i = 0; i < indices.size(); i++ )
            full_joints[indices[i]] = group_joints[i];
    }

    int Size() const {
        return indices.size();
    }
};
}
    