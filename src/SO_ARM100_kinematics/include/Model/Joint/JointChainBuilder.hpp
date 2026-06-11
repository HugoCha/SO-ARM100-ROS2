#pragma once

#include "Global.hpp"
#include "Joint.hpp"
#include "Link.hpp"

#include <memory>

namespace SOArm100::Kinematics::Model
{
class JointChain;

class JointChainBuilder
{
public:
JointChainBuilder& AddParentLink( 
    const std::string& name, 
    const Mat4d& home_tf );

JointChainBuilder& AddJoint( 
    const std::string& name,
    const Mat4d& joint_global_tf,
    const Twist& twist,
    const Limits& limits );

JointChainBuilder& AddChildLink( 
    const std::string& name, 
    const Mat4d& home_transform,
    const Mat4d& parent_joint_transform );

std::shared_ptr< const JointChain > Build();

private:
std::vector< Link > links_;
std::vector< Joint > joints_;
std::map< std::string, Link > link_map_;
std::map< std::string, Joint > joint_map_;
};
}