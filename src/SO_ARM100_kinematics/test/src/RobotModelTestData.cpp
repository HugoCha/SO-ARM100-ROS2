#include "RobotModelTestData.hpp"

#include <moveit/robot_model/robot_model.hpp>
#include <string>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>

namespace SOArm100::Kinematics::Test::Data
{

// ------------------------------------------------------------

moveit::core::RobotModelConstPtr revolute_robot_model_;
moveit::core::RobotModelPtr createRevoluteOnlyRobotModel();
const std::string createRevoluteOnlySRDFString();
const std::string createRevoluteOnlyRobotURDF();

// ------------------------------------------------------------

moveit::core::RobotModelPtr createRevoluteOnlyRobotModel()
{
	// Create URDF string for a simple robot with revolute joints only
	auto urdf_model = urdf::parseURDF( createRevoluteOnlyRobotURDF() );
	assert( urdf_model != nullptr );

	// Create SRDF model with arm group
	auto srdf_model = std::make_shared< srdf::Model >();
	srdf_model->initString( *urdf_model, createRevoluteOnlySRDFString() );

	// Create RobotModel from URDF and SRDF
	auto robot_model = std::make_shared< moveit::core::RobotModel >(
		urdf_model, srdf_model );
	assert( robot_model != nullptr );

	return robot_model;
}

// ------------------------------------------------------------

const std::string createRevoluteOnlySRDFString()
{
	return
	    R"(
                <?xml version="1.0"?>
                <robot name="revolute_only_robot">
                    <group name="arm">
                        <joint name="joint_1_revolute_z"/>
                        <joint name="joint_2_revolute_y"/>
                        <joint name="joint_3_revolute_z"/>
                    </group>
                </robot>
            )";
}

// ------------------------------------------------------------

const std::string createRevoluteOnlyRobotURDF()
{
	return
	    R"(
        <?xml version="1.0"?>
        <robot name="revolute_only_robot">
            <!-- Base link -->
            <link name="base_link">
                <inertial>
                    <mass value="1.0"/>
                    <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
                </inertial>
            </link>

            <!-- Joint 1: Revolute (rotation around z-axis) -->
            <joint name="joint_1_revolute_z" type="revolute">
                <parent link="base_link"/>
                <child link="link_1"/>
                <origin xyz="0 0 0" rpy="0 0 0"/>
                <axis xyz="0 0 1"/>
                <limit lower="-3.14159" upper="3.14159" effort="100" velocity="1.0"/>
            </joint>

            <link name="link_1">
                <inertial>
                    <mass value="1.0"/>
                    <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
                </inertial>
            </link>

            <!-- Joint 2: Revolute (rotation around y-axis) -->
            <joint name="joint_2_revolute_y" type="revolute">
                <parent link="link_1"/>
                <child link="link_2"/>
                <origin xyz="0.5 0 0" rpy="0 0 0"/>
                <axis xyz="0 1 0"/>
                <limit lower="-3.14159" upper="3.14159" effort="100" velocity="1.0"/>
            </joint>

            <link name="link_2">
                <inertial>
                    <mass value="1.0"/>
                    <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
                </inertial>
            </link>

            <!-- Joint 3: Revolute (rotation around z-axis) -->
            <joint name="joint_3_revolute_z" type="revolute">
                <parent link="link_2"/>
                <child link="end_effector"/>
                <origin xyz="0.5 0 0" rpy="0 0 0"/>
                <axis xyz="0 0 1"/>
                <limit lower="-3.14159" upper="3.14159" effort="100" velocity="1.0"/>
            </joint>

            <link name="end_effector">
                <inertial>
                    <mass value="0.5"/>
                    <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.005"/>
                </inertial>
            </link>
        </robot>
    )";
}

// ------------------------------------------------------------

moveit::core::RobotModelConstPtr GetRevoluteOnlyRobot()
{
	if ( !revolute_robot_model_ )
	{
		revolute_robot_model_ = createRevoluteOnlyRobotModel();
	}
	return revolute_robot_model_;
}

// ------------------------------------------------------------

}