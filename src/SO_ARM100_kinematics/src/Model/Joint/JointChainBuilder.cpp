#include "Model/Joint/JointChainBuilder.hpp"

#include "Global.hpp"
#include "Model/Joint/JointChain.hpp"
#include "Model/Joint/Link.hpp"
#include "Utils/KinematicsUtils.hpp"

#include <memory>
#include <stdexcept>
#include <utility>

namespace SOArm100::Kinematics::Model
{

// ------------------------------------------------------------

JointChainBuilder& JointChainBuilder::AddParentLink( 
    const std::string& name, 
    const Mat4d& link_global_tf )
{
    if ( !link_map_.contains( name ) )
    {
        auto link = Link( name, link_global_tf, Mat4d::Identity(), 0.0 );
        link_map_.emplace( std::make_pair( name, link ) );
        links_.emplace_back( link );
    }
    return *this;
}

// ------------------------------------------------------------

JointChainBuilder& JointChainBuilder::
AddJoint( 
    const std::string& name,
    const Mat4d& joint_global_tf,
    const Twist& twist,
    const Limits& limits )
{
    if ( !joint_map_.contains( name ) )
    {
        auto joint = Joint(
            name,
            joint_global_tf,
            twist,
            limits,
            nullptr,
            nullptr );

        joint_map_.emplace( std::make_pair( name, joint ) );
        joints_.emplace_back( joint );
    }
    return *this;
}

// ------------------------------------------------------------

JointChainBuilder& JointChainBuilder::AddChildLink( 
    const std::string& name, 
    const Mat4d& home_transform,
    const Mat4d& parent_joint_transform )
{
    if ( !link_map_.contains( name ) )
    {
        auto link = Link(
            name,
            home_transform,
            parent_joint_transform,
            0.0 );

        link_map_.emplace( std::make_pair( name, link ) );
        links_.emplace_back( link );
    }
    return *this;
}

// ------------------------------------------------------------

std::shared_ptr< const JointChain > JointChainBuilder::Build()
{
    if ( links_.size() != joints_.size() + 1 )
        throw std::invalid_argument( "Links size must be equal to joints size + 1" );

    std::vector< LinkConstPtr > links;
    std::vector< JointConstPtr > joints;

    auto root_link = links_[0];
    double root_link_length = 0.0;

    if ( !joints_.empty() )
        root_link_length = ( joints_[0].Origin() - Translation( root_link.HomeTransform() ) ).norm();

    links.emplace_back( std::make_shared< const Link >( 
        root_link.GetName(), 
        root_link.HomeTransform(), 
        root_link.ParentJointTransform(), 
        root_link_length ) );

    for ( int i = 0; i < joints_.size() - 1; i++ )
    {
        auto parent_link_ptr = links[i];
        auto joint = joints_[i];
        auto child_link = links_[i + 1];
        auto child_joint = joints_[i + 1];
        
        double link_length = ( child_joint.Origin() - joint.Origin() ).norm();
        auto child_link_ptr = std::make_shared< const Link >( 
            child_link.GetName(), 
            child_link.HomeTransform(), 
            child_link.ParentJointTransform(), 
            link_length );
            
        auto joint_ptr = std::make_shared< const Joint >(
            joint.GetName(),
            joint.OriginTransform(),
            joint.GetTwist(),
            joint.GetLimits(),
            parent_link_ptr,
            child_link_ptr );

        joints.emplace_back( joint_ptr );
        links.emplace_back( child_link_ptr );
    }

    if ( !joints_.empty() )
    {
        LinkConstPtr parent_link_ptr = links.back();
        auto last_joint = joints_.back();
        auto tip_link = links_.back();
        double tip_link_length = Translation( tip_link.ParentJointTransform() ).norm();
        auto tip_link_ptr = std::make_shared< const Link >(
            tip_link.GetName(), 
            tip_link.HomeTransform(), 
            tip_link.ParentJointTransform(), 
            tip_link_length );

        auto last_joint_ptr = std::make_shared< const Joint >(
            last_joint.GetName(),
            last_joint.OriginTransform(),
            last_joint.GetTwist(),
            last_joint.GetLimits(),
            parent_link_ptr,
            tip_link_ptr );

        joints.emplace_back( last_joint_ptr );
        links.emplace_back( tip_link_ptr );
    }

    return std::shared_ptr< JointChain >( new JointChain( joints, links ) );
}

// ------------------------------------------------------------

}