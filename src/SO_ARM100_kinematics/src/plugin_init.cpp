#include <pluginlib/class_list_macros.hpp>

#include "RobotArmKinematicsPlugin.hpp"

PLUGINLIB_EXPORT_CLASS(
    SOArm100::Kinematics::RobotArmKinematicsPlugin,
    kinematics::KinematicsBase)