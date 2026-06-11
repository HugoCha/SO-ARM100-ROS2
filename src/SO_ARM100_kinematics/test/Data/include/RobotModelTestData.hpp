#pragma once

#include "Global.hpp"
#include "Model/Joint/JointChain.hpp"
#include "Model/KinematicModel.hpp"

#include <moveit/macros/class_forward.hpp>
#include <map>

namespace moveit::core
{
MOVEIT_CLASS_FORWARD( RobotModel );
}

namespace SOArm100::Kinematics::Model
{
class JointChain;
};

namespace SOArm100::Kinematics::Test
{
class Data
{
public:
// ZYZ Revolute Robot
static const Mat4d GetZYZRevoluteRobotT01( double theta1 );
static const Mat4d GetZYZRevoluteRobotT12( double theta2 );
static const Mat4d GetZYZRevoluteRobotT23( double theta3 );
static Mat4d GetZYZRevoluteRobotTransform( double theta1, double theta2, double theta3 );
static MatXd GetZYZRevoluteRobotJacobian( double theta1, double theta2, double theta3 );

static moveit::core::RobotModelConstPtr GetZYZRevoluteRobotMoveitModel();
static Mat4d GetZYZRevoluteRobotHome();
static const Model::JointChain* GetZYZRevoluteRobotJointChain();
static Model::KinematicTopology GetZYZRevoluteRobotTopology();
static Model::KinematicModelConstPtr GetZYZRevoluteRobot();
static std::string GetZYZRevoluteRobotName() {
    return "ZYZ";
}

// Revolute Base Robot
static Model::KinematicModelConstPtr GetRevoluteBaseRobot();
static std::string GetRevoluteBaseRobotName() {
    return "RevoluteBase";
}

// Prismatic Base Robot
static Model::KinematicModelConstPtr GetPrismaticBaseRobot();
static std::string GetPrismaticBaseRobotName() {
    return "PrismaticBase";
}

// Planar2R Robot
static Model::KinematicModelConstPtr GetPlanar2RRobot();
static std::string GetPlanar2RRobotName() {
    return "Planar2R";
}

// Planar3R Robot
static Model::KinematicModelConstPtr GetPlanar3RRobot();
static std::string GetPlanar3RRobotName() {
    return "Planar3R";
}

// 1-DOF Wrist Robot
static Model::KinematicModelConstPtr GetWrist1RRobot();
static std::string GetWrist1RRobotName() {
    return "Wrist1R";
}

// 2-DOFs Wrist Robot
static Model::KinematicModelConstPtr GetWrist2RRobot();
static std::string GetWrist2RRobotName() {
    return "Wrist2R";
}

// Spherical Wrist Robot
static Model::KinematicModelConstPtr GetSphericalWristRobot();
static std::string GetSphericalWristRobotName() {
    return "Wrist3R";
}

// Revolute Base - Planar2R - Wrist2R / 5 DOFs
static Model::KinematicModelConstPtr GetRevolute_Planar2R_Wrist2R_5DOFsRobot();
static std::string GetRevolute_Planar2R_Wrist2R_5DOFsRobotName() {
    return "5-axis arm";
}

// Revolute Base - Planar2R - Spherical Wrist / 6 DOFs
static Model::KinematicModelConstPtr GetRevolute_Planar2R_SphericalWrist_6DOFsRobot();
static std::string GetRevolute_Planar2R_SphericalWrist_6DOFsRobotName() {
    return "6-axis arm";
}

// Revolute Base - Planar 2R - Non Spherical Wrist / 6 DOFs
static Model::KinematicModelConstPtr GetURLikeRobot();
static std::string GetURLikeRobotName() {
    return "Universal Robot";
}

static Model::KinematicModelConstPtr GetLeRobot();
static std::string GetLeRobotName() {
    return "LeRobot";
}
static Model::KinematicModelConstPtr GetLeRobotWithGripper();

static std::map< std::string, Model::KinematicModelConstPtr > GetAllRobots();
};
}