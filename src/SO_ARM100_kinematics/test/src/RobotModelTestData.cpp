#include "RobotModelTestData.hpp"

#include "Global.hpp"
#include "JointChain.hpp"
#include "KinematicsUtils.hpp"
#include "Limits.hpp"
#include "Link.hpp"

#include <cmath>
#include <Eigen/Dense>
#include <memory>
#include <moveit/robot_model/joint_model.hpp>
#include <moveit/robot_model/link_model.hpp>
#include <moveit/robot_model/prismatic_joint_model.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>
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
const Mat4d GetRevoluteOnlyRobotT01( double theta1 );
const Mat4d GetRevoluteOnlyRobotT12( double theta2 );
const Mat4d GetRevoluteOnlyRobotT23( double theta3 );

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

const Mat4d GetRevoluteOnlyRobotT01( double theta1 )
{
	Mat4d T01 = Mat4d::Identity();

	T01.block< 3, 3 >( 0, 0 ) =
		Eigen::AngleAxisd( theta1, Vec3d::UnitZ() ).toRotationMatrix();
	T01.block< 3, 1 >( 0, 3 ) = Vec3d::Zero();

	return T01;
}

// ------------------------------------------------------------

const Mat4d GetRevoluteOnlyRobotT12( double theta2 )
{
	Mat4d T12 = Mat4d::Identity();

	T12.block< 3, 3 >( 0, 0 ) =
		Eigen::AngleAxisd( theta2, Vec3d::UnitY() ).toRotationMatrix();
	T12.block< 3, 1 >( 0, 3 ) = Vec3d( 0.5, 0, 0 );

	return T12;
}

// ------------------------------------------------------------

const Mat4d GetRevoluteOnlyRobotT23( double theta3 )
{
	Mat4d T23 = Mat4d::Identity();

	T23.block< 3, 3 >( 0, 0 ) =
		Eigen::AngleAxisd( theta3, Vec3d::UnitZ() ).toRotationMatrix();
	T23.block< 3, 1 >( 0, 3 ) = Vec3d( 0.5, 0, 0 );

	return T23;
}

// ------------------------------------------------------------

Mat4d GetRevoluteOnlyRobotTransform( double theta1, double theta2, double theta3 )
{
	Mat4d T01 = GetRevoluteOnlyRobotT01( theta1 );
	Mat4d T12 = GetRevoluteOnlyRobotT12( theta2 );
	Mat4d T23 = GetRevoluteOnlyRobotT23( theta3 );
	return T01 * T12 * T23;
}

// ------------------------------------------------------------

const JointChain GetRevoluteOnlyRobotJointChain()
{
	JointChain joint_chain( 3 );
	// // Configuration HOME (tous les angles à 0)
	// Joint 1: Axe Z à l'origine
	Vec3d axis1( 0, 0, 1 );           // Axe Z
	Vec3d point1( 0, 0, 0 );          // Origine
	Twist twist1( axis1, point1 );

	Mat4d origin1 = Mat4d::Identity();
	Link link1( origin1 );

	Limits limits1( -M_PI, M_PI );

	joint_chain.Add( twist1, link1, limits1 );

	// Joint 2: Axe Y après translation de 0.5m en X
	// À la home: le joint 2 est en (0.5, 0, 0) avec axe Y
	Vec3d axis2( 0, 1, 0 );           // Axe Y dans repère spatial
	Vec3d point2( 0.5, 0, 0 );        // Position à home
	Twist twist2( axis2, point2 );

	Mat4d origin2 = Mat4d::Identity();
	origin2( 0, 3 ) = 0.5;
	Link link2( origin2 );

	Limits limits2( -M_PI, M_PI );

	joint_chain.Add( twist2, link2, limits2 );

	// Joint 3: Axe Z après translation totale de 1.0m en X
	// À la home: le joint 3 est en (1.0, 0, 0) avec axe Z
	Vec3d axis3( 0, 0, 1 );           // Axe Z dans repère spatial
	Vec3d point3( 1.0, 0, 0 );        // Position à home
	Twist twist3( axis3, point3 );

	Mat4d origin3 = Mat4d::Identity();
	origin3( 0, 3 ) = 1.0;
	Link link3( origin3 );

	Limits limits3( -M_PI, M_PI );

	joint_chain.Add( twist3, link3, limits3 );

	return joint_chain;
}

// ------------------------------------------------------------

MatXd GetRevoluteOnlyRobotJacobian( double theta1, double theta2, double theta3 )
{
	MatXd J( 6, 3 );

	const Mat4d& T01 = GetRevoluteOnlyRobotT01( theta1 );
	const Mat4d& T12 = GetRevoluteOnlyRobotT12( theta2 );
	const Mat4d& T23 = GetRevoluteOnlyRobotT23( theta3 );
	const Mat4d& T02 = T01 * T12;
	const Mat4d& T03 = T02 * T23;

	const Mat3d R01 = Rotation( T01 );
	const Mat3d R02 = Rotation( T02 );

	Vec3d z0( 0, 0, 1 );
	Vec3d y1 = R01 * Vec3d( 0, 1, 0 );
	Vec3d z2 = R02 * z0;

	Vec3d p0 = Translation( T01 );
	Vec3d p1 = Translation( T02 );
	Vec3d p2 = Translation( T03 );

	Vec3d pe = Translation( T03 );

	J.block< 3, 1 >( 0, 0 ) = z0;
	J.block< 3, 1 >( 0, 1 ) = y1;
	J.block< 3, 1 >( 0, 2 ) = z2;

	J.block< 3, 1 >( 3, 0 ) = z0.cross( pe - p0 );
	J.block< 3, 1 >( 3, 1 ) = y1.cross( pe - p1 );
	J.block< 3, 1 >( 3, 2 ) = z2.cross( pe - p2 );

	return J;
}

// ------------------------------------------------------------

}