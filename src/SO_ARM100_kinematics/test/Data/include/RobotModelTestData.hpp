#pragma once

#include "Global.hpp"
#include "Model/KinematicModel.hpp"

#include <moveit/macros/class_forward.hpp>

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
static Model::JointChain GetZYZRevoluteRobotJointChain();
static Model::KinematicTopology GetZYZRevoluteRobotTopology();
static Model::KinematicModelConstPtr GetZYZRevoluteRobot();

// Revolute Base Robot
static Model::KinematicModelConstPtr GetRevoluteBaseRobot();

// Prismatic Base Robot
static Model::KinematicModelConstPtr GetPrismaticBaseRobot();

// Planar2R Robot
static Model::KinematicModelConstPtr GetPlanar2RRobot();

// Planar3R Robot
static Model::KinematicModelConstPtr GetPlanar3RRobot();

// 1-DOF Wrist Robot
static Model::KinematicModelConstPtr GetWrist1RRobot();

// 2-DOFs Wrist Robot
static Model::KinematicModelConstPtr GetWrist2RRobot();

// Spherical Wrist Robot
static Model::KinematicModelConstPtr GetSphericalWristRobot();

// Revolute Base - Planar2R - Wrist2R / 5 DOFs
static Model::KinematicModelConstPtr GetRevolute_Planar2R_Wrist2R_5DOFsRobot();

// Revolute Base - Planar2R - Spherical Wrist / 6 DOFs
static Model::KinematicModelConstPtr GetRevolute_Planar2R_Wrist3R_6DOFsRobot();
};
}