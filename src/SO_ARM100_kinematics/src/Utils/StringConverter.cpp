#include "Utils/StringConverter.hpp"

#include "Global.hpp"

#include "Model/Joint/Joint.hpp"
#include "Model/Joint/JointChain.hpp"
#include "Model/Joint/JointState.hpp"
#include "Model/Joint/JointType.hpp"
#include "Model/Limits.hpp"
#include "Model/Link.hpp"
#include "Model/Pose.hpp"
#include "Model/Skeleton/Articulation.hpp"
#include "Model/Skeleton/ArticulationState.hpp"
#include "Model/Skeleton/ArticulationType.hpp"
#include "Model/Skeleton/Bone.hpp"
#include "Model/Skeleton/BoneState.hpp"
#include "Model/Skeleton/Skeleton.hpp"

// ------------------------------------------------------------

std::ostream& operator<<( std::ostream& os, const SOArm100::Kinematics::Model::Articulation& obj )
{
    os << "Articulation " 
       << obj.GetType()
       << " Center: " << obj.Center().transpose()
       << " Axis: " << obj.Axis().transpose();
    return os;
}

// ------------------------------------------------------------

std::ostream& operator<<(std::ostream& os, const SOArm100::Kinematics::Model::ArticulationState& obj )
{
    os << "Articulation state"
       << " Center: " << obj.Origin().transpose()
       << " Axis: "   << obj.Axis().transpose()
       << " Value: "  << obj.Value(); 
    return os;
}

// ------------------------------------------------------------

std::ostream& operator<<(std::ostream& os, const SOArm100::Kinematics::Model::ArticulationType& obj )
{
	switch ( obj )
	{
		case SOArm100::Kinematics::Model::ArticulationType::Fixed:     { os << "FIXED"; return os; }
		case SOArm100::Kinematics::Model::ArticulationType::Revolute:  { os << "REVOLUTE"; return os; }
		case SOArm100::Kinematics::Model::ArticulationType::Prismatic: { os << "PRISMATIC"; return os; }
		case SOArm100::Kinematics::Model::ArticulationType::Universal: { os << "UNIVERSAL"; return os; }
		case SOArm100::Kinematics::Model::ArticulationType::Spherical: { os << "SPHERICAL"; return os; }
	}
    return os;
}

// ------------------------------------------------------------

std::ostream& operator<<(std::ostream& os, const SOArm100::Kinematics::Model::Bone& obj )
{
    os << "Bone" 
       << " Origin: " << obj.Origin().transpose()
       << " Direction: " << obj.Direction().transpose()
       << " Length: " << obj.Length();
    return os;
}

// ------------------------------------------------------------

std::ostream& operator<<(std::ostream& os, const SOArm100::Kinematics::Model::BoneState& obj )
{
    os << "Bone State" 
       << " Origin: " << obj.Origin().transpose()
       << " Direction: " << obj.Direction().transpose();
    return os;
}

// ------------------------------------------------------------

std::ostream& operator<<(std::ostream& os, const SOArm100::Kinematics::Model::Joint& obj )
{
    os << "Joint "
       << obj.GetType() 
       << " " << obj.GetTwist()
       << " " << obj.GetLink()
       << " " << obj.GetLimits();
    return os;
}

// ------------------------------------------------------------

std::ostream& operator<<(std::ostream& os, const SOArm100::Kinematics::Model::JointChain& obj )
{
    os << "Joint Chain" << std::endl;
    for ( int i = 0; i < obj.GetJointCount(); i++ )
    {
        os << std::to_string( i + 1 ) << ": " << *obj.GetJoints()[i];
        if ( i < obj.GetJointCount() - 1 )
            os << std::endl;
    }
    return os;
}

// ------------------------------------------------------------

std::ostream& operator<<(std::ostream& os, const SOArm100::Kinematics::Model::JointState& obj )
{
    os << "Joint State"
       << " Origin: " << obj.Origin().transpose()
       << " Axis: " << obj.Axis().transpose() 
       << " Value: " << obj.Value();
    return os;
}

// ------------------------------------------------------------

std::ostream& operator<<(std::ostream& os, const SOArm100::Kinematics::Model::JointType& obj )
{
	switch ( obj )
	{
		case SOArm100::Kinematics::Model::JointType::FIXED: 	{ os << "FIXED"; return os; }
		case SOArm100::Kinematics::Model::JointType::REVOLUTE:  { os << "REVOLUTE"; return os; }
		case SOArm100::Kinematics::Model::JointType::PRISMATIC: { os << "PRISMATIC"; return os; }
	}
    return os;
}

// ------------------------------------------------------------

std::ostream& operator<<(std::ostream& os, const SOArm100::Kinematics::Model::Limits& obj )
{
    os << "Limits"
       << " Min: " << obj.Min()
       << " Max: " << obj.Max();
    return os;
}  

// ------------------------------------------------------------

std::ostream& operator<<(std::ostream& os, const SOArm100::Kinematics::Model::Link& obj )
{
    os << "Link"
       << " Origin: " << obj.GetJointOrigin().transpose()
       << " Length: " << obj.GetLength();
    return os;
} 

// ------------------------------------------------------------

std::ostream& operator<<(std::ostream& os, const SOArm100::Kinematics::Model::Pose& obj )
{
    os << "Pose"
       << " Origin: " << obj.origin.transpose()
       << " Axis: " << obj.axis.transpose();
    return os;
}

// ------------------------------------------------------------

std::ostream& operator<<(std::ostream& os, const SOArm100::Kinematics::Model::Skeleton& obj )
{
    os << "Skeleton" << std::endl;
    for ( int i = 0; i < obj.ArticulationCount(); i++ )
    {
        os << std::to_string( i + 1 ) << ": " << *obj.Articulation( i ) << std::endl;
        if ( i < obj.BonesCount() )
            os << std::to_string( i + 1 ) << ": " << *obj.Bone( i ) << std::endl;
    }
    os << "Total Length: " << obj.TotalLength();
    return os;
} 

// ------------------------------------------------------------

std::ostream& operator<<(std::ostream& os, const SOArm100::Kinematics::Model::SkeletonState& obj )
{
    os << "Skeleton State" << std::endl;
    return os;
}

// ------------------------------------------------------------

std::ostream& operator<<(std::ostream& os, const SOArm100::Kinematics::Model::Twist& obj )
{
    os << "Twist"
       << " w: " << obj.Omega().transpose()
       << " v: " << obj.V().transpose();
    return os;  
}

// ------------------------------------------------------------