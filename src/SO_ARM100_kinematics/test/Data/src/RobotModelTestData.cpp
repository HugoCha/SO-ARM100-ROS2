#include "RobotModelTestData.hpp"

#include "Global.hpp"

#include "Model/Joint/Joint.hpp"
#include "Model/Joint/JointChain.hpp"
#include "Model/Joint/JointGroup.hpp"
#include "Model/Joint/Limits.hpp"
#include "Model/Joint/Link.hpp"
#include "Model/Joint/Twist.hpp"
#include "Model/KinematicModel.hpp"
#include "Model/KinematicTopology/KinematicTopology.hpp"
#include "Model/ReachableSpace/ReachableSpace.hpp"
#include "Model/ReachableSpace/ChainTotalLengthReachableSpace.hpp"
#include "Model/ReachableSpace/SkeletonTotalLengthReachableSpace.hpp"
#include "Model/Skeleton/Articulation.hpp"
#include "Model/Skeleton/ArticulationType.hpp"
#include "Model/Skeleton/Skeleton.hpp"
#include "ModelAnalyzer/SkeletonAnalyzer.hpp"
#include "Utils/Converter.hpp"
#include "Utils/KinematicsUtils.hpp"

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

namespace SOArm100::Kinematics::Test
{

// ------------------------------------------------------------
// ------------------------------------------------------------

moveit::core::RobotModelPtr ZYZRevolute_moveit_robot_ = nullptr;
Model::KinematicModelConstPtr ZYZRevolute_robot_ = nullptr;
Model::KinematicModelConstPtr Planar2R_robot_ = nullptr;
Model::KinematicModelConstPtr Planar3R_robot_ = nullptr;
Model::KinematicModelConstPtr Wrist1R_robot_ = nullptr;
Model::KinematicModelConstPtr Wrist2R_robot_ = nullptr;
Model::KinematicModelConstPtr SphericalWrist_robot_ = nullptr;
Model::KinematicModelConstPtr RevoluteBase_robot_ = nullptr;
Model::KinematicModelConstPtr PrismaticBase_robot_ = nullptr;
Model::KinematicModelConstPtr Revolute_Planar2R_Wrist2R_5DOFs_robot_ = nullptr;
Model::KinematicModelConstPtr Revolute_Planar2R_SphericalWrist_6DOFs_robot_ = nullptr;
Model::KinematicModelConstPtr URLike_robot_ = nullptr;

std::unique_ptr< Model::ReachableSpace > createChainReachableSpacePtr(
	const Model::JointChain& chain,
	const Mat4d home )
{
	return std::make_unique< Model::ChainTotalLengthReachableSpace >( chain, home );
}

std::unique_ptr< Model::ReachableSpace > createSkeletonReachableSpacePtr(
	const Model::Skeleton& skeleton )
{
	return std::make_unique< Model::SkeletonTotalLengthReachableSpace >( skeleton );
}

// ------------------------------------------------------------
// ZYZ Revolute Robot
// ------------------------------------------------------------

moveit::core::RobotModelPtr createZYZRevoluteRobotModel();
const std::string createZYZRevoluteSRDFString();
const std::string createZYZRevoluteRobotURDF();

// ------------------------------------------------------------

moveit::core::RobotModelPtr createZYZRevoluteRobotModel()
{
	// Create URDF string for a simple robot with revolute joints only
	auto urdf_model = urdf::parseURDF( createZYZRevoluteRobotURDF() );
	assert( urdf_model != nullptr );

	// Create SRDF model with arm group
	auto srdf_model = std::make_shared< srdf::Model >();
	srdf_model->initString( *urdf_model, createZYZRevoluteSRDFString() );

	// Create RobotModel from URDF and SRDF
	auto robot_model = std::make_shared< moveit::core::RobotModel >(
		urdf_model, srdf_model );
	assert( robot_model != nullptr );

	return robot_model;
}

// ------------------------------------------------------------

const std::string createZYZRevoluteSRDFString()
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

const std::string createZYZRevoluteRobotURDF()
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

moveit::core::RobotModelConstPtr Data::GetZYZRevoluteRobotMoveitModel()
{
	if ( !ZYZRevolute_moveit_robot_ )
	{
		ZYZRevolute_moveit_robot_ = createZYZRevoluteRobotModel();
	}
	return ZYZRevolute_moveit_robot_;
}

// ------------------------------------------------------------

const Mat4d Data::GetZYZRevoluteRobotT01( double theta1 )
{
	Mat4d T01 = Mat4d::Identity();

	T01.block< 3, 3 >( 0, 0 ) =
		AngleAxis( theta1, Vec3d::UnitZ() ).toRotationMatrix();
	T01.block< 3, 1 >( 0, 3 ) = Vec3d( 0.0, 0, 0 );

	return T01;
}

// ------------------------------------------------------------

const Mat4d Data::GetZYZRevoluteRobotT12( double theta2 )
{
	Mat4d T12 = Mat4d::Identity();

	T12.block< 3, 3 >( 0, 0 ) =
		AngleAxis( theta2, Vec3d::UnitY() ).toRotationMatrix();
	T12.block< 3, 1 >( 0, 3 ) = Vec3d( 0.5, 0, 0 );

	return T12;
}

// ------------------------------------------------------------

const Mat4d Data::GetZYZRevoluteRobotT23( double theta3 )
{
	Mat4d T23 = Mat4d::Identity();

	T23.block< 3, 3 >( 0, 0 ) =
		AngleAxis( theta3, Vec3d::UnitZ() ).toRotationMatrix();
	T23.block< 3, 1 >( 0, 3 ) = Vec3d( 0.5, 0, 0 );

	return T23;
}

// ------------------------------------------------------------

Mat4d Data::GetZYZRevoluteRobotTransform( double theta1, double theta2, double theta3 )
{
	Mat4d T01 = Data::GetZYZRevoluteRobotT01( theta1 );
	Mat4d T12 = Data::GetZYZRevoluteRobotT12( theta2 );
	Mat4d T23 = Data::GetZYZRevoluteRobotT23( theta3 );
	return T01 * T12 * T23;
}

// ------------------------------------------------------------

Model::JointChain createZYZRevoluteRobotJointChain()
{
	Model::JointChain joint_chain( 3 );
	// // Configuration HOME (tous les angles à 0)
	// Joint 1: Axe Z à l'origine
	Vec3d axis1( 0, 0, 1 );           // Axe Z
	Vec3d point1( 0, 0, 0 );          // Origine
	Model::Twist twist1( axis1, point1 );

	Vec3d origin1 = Vec3d( 0.0, 0, 0.0 );
	Vec3d origin2 = Vec3d( 0.5, 0, 0.0 );
	Vec3d origin3 = Vec3d( 1.0, 0, 0.0 );
	Model::Link link1( ToTransformMatrix( origin1 ), 0.5 );

	Model::Limits limits1( -M_PI, M_PI );

	joint_chain.Add( twist1, link1, limits1 );

	// Joint 2: Axe Y après translation de 0.5m en X
	// À la home: le joint 2 est en (0.5, 0, 0) avec axe Y
	Vec3d axis2( 0, 1, 0 );           // Axe Y dans repère spatial
	Vec3d point2( 0.5, 0, 0 );        // Position à home
	Model::Twist twist2( axis2, point2 );

	Model::Link link2( ToTransformMatrix( origin2 ), 0.5 );

	Model::Limits limits2( -M_PI, M_PI );

	joint_chain.Add( twist2, link2, limits2 );

	// Joint 3: Axe Z après translation totale de 1.0m en X
	// À la home: le joint 3 est en (1.0, 0, 0) avec axe Z
	Vec3d axis3( 0, 0, 1 );           // Axe Z dans repère spatial
	Vec3d point3( 1.0, 0, 0 );        // Position à home
	Model::Twist twist3( axis3, point3 );

	Model::Link link3( ToTransformMatrix( origin3 ), 0 );

	Model::Limits limits3( -M_PI, M_PI );

	joint_chain.Add( twist3, link3, limits3 );

	return joint_chain;
}

// ------------------------------------------------------------

Mat4d createZYZRevoluteRobotHome()
{
	Mat4d home;
	home << 1, 0, 0, 1,
	    0, 1, 0, 0,
	    0, 0, 1, 0,
	    0, 0, 0, 1;
	return home;
}

// ------------------------------------------------------------

Model::KinematicTopology createZYZRevoluteRobotTopology()
{
	Model::KinematicTopology topology;
	Mat4d home = createZYZRevoluteRobotHome();

	Model::RevoluteBaseJointGroup base_group( home );
	Model::WristJointGroup wrist_group( 2, 1, home );

	topology.Add( base_group );
	topology.Add( wrist_group );

	return topology;
}

// ------------------------------------------------------------

Model::Skeleton createZYZRevoluteSkeleton(
	const Model::JointChain& chain,
	const Mat4d& home )
{
	std::vector< Model::ArticulationConstPtr > articulations;
	std::vector< Model::BoneConstPtr > bones;

	articulations.emplace_back(
		std::make_shared< const Model::Articulation >(
			Model::ArticulationType::Revolute,
			std::vector< Model::JointConstPtr > { chain.GetActiveJoint( 0 ) },
			chain.GetActiveJoint( 0 )->Origin()
			)
		);

	articulations.emplace_back(
		std::make_shared< const Model::Articulation >(
			Model::ArticulationType::Revolute,
			std::vector< Model::JointConstPtr > { chain.GetActiveJoint( 1 ) },
			chain.GetActiveJoint( 1 )->Origin()
			)
		);

	articulations.emplace_back(
		std::make_shared< const Model::Articulation >(
			Model::ArticulationType::Revolute,
			std::vector< Model::JointConstPtr > { chain.GetActiveJoint( 2 ) },
			chain.GetActiveJoint( 2 )->Origin()
			)
		);

	bones.emplace_back(
		std::make_shared< const Model::Bone >(
			chain.GetActiveJoint( 0 )->Origin(),
			chain.GetActiveJoint( 1 )->Origin() - chain.GetActiveJoint( 0 )->Origin() )
		);

	bones.emplace_back(
		std::make_shared< const Model::Bone >(
			chain.GetActiveJoint( 1 )->Origin(),
			chain.GetActiveJoint( 2 )->Origin() - chain.GetActiveJoint( 1 )->Origin() )
		);

	double total_length = 0.0;
	for ( int i = 0; i < bones.size(); i++ )
		total_length += bones[i]->Length();

	return Model::Skeleton( articulations, bones, total_length, chain.GetJointCount() );
}

// ------------------------------------------------------------

Model::KinematicModelConstPtr createZYZRevoluteRobot()
{
	auto joint_chain = std::make_unique< const Model::JointChain >( createZYZRevoluteRobotJointChain() );
	auto home = createZYZRevoluteRobotHome();
	auto topology = createZYZRevoluteRobotTopology();
	auto skeleton = std::make_shared< const Model::Skeleton >( createZYZRevoluteSkeleton( *joint_chain, home ) );
	auto reachable_space = createSkeletonReachableSpacePtr( *skeleton );
	return std::make_shared< const Model::KinematicModel >(
		std::move( joint_chain ),
		home,
		topology,
		skeleton,
		std::move( reachable_space ) );
}

// ------------------------------------------------------------

Model::KinematicModelConstPtr Data::GetZYZRevoluteRobot()
{
	if ( !ZYZRevolute_robot_ )
	{
		ZYZRevolute_robot_ = createZYZRevoluteRobot();
	}
	return ZYZRevolute_robot_;
}

// ------------------------------------------------------------

Model::JointChain Data::GetZYZRevoluteRobotJointChain()
{
	return *GetZYZRevoluteRobot()->GetChain();
}

// ------------------------------------------------------------

Mat4d Data::GetZYZRevoluteRobotHome()
{
	return GetZYZRevoluteRobot()->GetHomeConfiguration();
}

// ------------------------------------------------------------

Model::KinematicTopology Data::GetZYZRevoluteRobotTopology()
{
	return GetZYZRevoluteRobot()->GetTopology();
}

// ------------------------------------------------------------

MatXd Data::GetZYZRevoluteRobotJacobian( double theta1, double theta2, double theta3 )
{
	MatXd J( 6, 3 );

	const Mat4d& T01 = Data::GetZYZRevoluteRobotT01( theta1 );
	const Mat4d& T12 = Data::GetZYZRevoluteRobotT12( theta2 );
	const Mat4d& T23 = Data::GetZYZRevoluteRobotT23( theta3 );
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
// Revolute Base Robot
// ------------------------------------------------------------

std::unique_ptr< const Model::JointChain > createRevoluteBaseJointChain()
{
	auto chain = std::make_unique< Model::JointChain >( 3 );

	chain->Add(
		{ Vec3d::UnitZ(), Vec3d::Zero() },
		{ Mat4d::Identity(), 1 },
		{ -M_PI, M_PI }
		);

	chain->Add(
		{ Vec3d::UnitY(), Vec3d( 0, 0, 1 ) },
		{ ToTransformMatrix( Vec3d( 0, 0, 1 ) ), 1 },
		{ -M_PI, M_PI }
		);

	chain->Add(
		{ Vec3d::UnitZ(), Vec3d( 1, 0, 1 ) },
		{ ToTransformMatrix( Vec3d( 1, 0, 1 ) ), 0.1 },
		{ -M_PI, M_PI }
		);

	return chain;
}

// ------------------------------------------------------------

Mat4d createRevoluteBaseHome()
{
	return ToTransformMatrix( Vec3d( 1, 0, 0.9 ) );
}

// ------------------------------------------------------------

Model::KinematicTopology createRevoluteBaseTopology( const Mat4d& home )
{
	Model::KinematicTopology topology;

	Model::RevoluteBaseJointGroup base_group(
		ToTransformMatrix( Vec3d( 1, 0, 1 ) ) );

	Model::WristJointGroup wrist_group(
		1,
		2,
		ToTransformMatrix( Vec3d( 0, 0, 0.1 ) ) );

	topology.Add( base_group );
	topology.Add( wrist_group );

	return topology;
}

// ------------------------------------------------------------

Model::Skeleton createRevoluteBaseSkeleton(
	const Model::JointChain& chain,
	const Mat4d& home )
{
	std::vector< Model::ArticulationConstPtr > articulations;
	std::vector< Model::BoneConstPtr > bones;

	articulations.emplace_back(
		std::make_shared< const Model::Articulation >(
			Model::ArticulationType::Universal,
			std::vector< Model::JointConstPtr > { chain.GetActiveJoint( 0 ), chain.GetActiveJoint( 1 ) },
			chain.GetActiveJoint( 1 )->Origin()
			)
		);

	articulations.emplace_back(
		std::make_shared< const Model::Articulation >(
			Model::ArticulationType::Revolute,
			std::vector< Model::JointConstPtr > { chain.GetActiveJoint( 2 ) },
			Translation( home )
			)
		);

	bones.emplace_back(
		std::make_shared< const Model::Bone >(
			articulations[0]->Center(),
			Translation( home ) - articulations[0]->Center() )
		);

	double total_length = articulations[0]->Center().norm();
	for ( int i = 0; i < bones.size(); i++ )
		total_length += bones[i]->Length();

	return Model::Skeleton( articulations, bones, total_length, chain.GetJointCount() );
}

// ------------------------------------------------------------

Model::KinematicModelConstPtr createRevoluteBaseRobot()
{
	auto chain = createRevoluteBaseJointChain();
	auto home = createRevoluteBaseHome();
	auto topology = createRevoluteBaseTopology( home );
	auto skeleton = std::make_shared< const Model::Skeleton >( createRevoluteBaseSkeleton( *chain, home ) );
	auto reachable_space = createSkeletonReachableSpacePtr( *skeleton );
	return std::make_shared< const Model::KinematicModel >(
		std::move( chain ),
		home,
		topology,
		skeleton,
		std::move( reachable_space ) );
}

// ------------------------------------------------------------

Model::KinematicModelConstPtr Data::GetRevoluteBaseRobot()
{
	if ( !RevoluteBase_robot_ )
	{
		RevoluteBase_robot_ = createRevoluteBaseRobot();
	}
	return RevoluteBase_robot_;
}

// ------------------------------------------------------------
// Prismatic Base Robot
// ------------------------------------------------------------

std::unique_ptr< const Model::JointChain > createPrismaticBaseJointChain()
{
	auto chain = std::make_unique< Model::JointChain >( 1 );

	chain->Add(
		{ Vec3d::UnitX() },
		{ Mat4d::Identity(), 1 },
		{ 0, 1 }
		);

	chain->Add(
		{ Vec3d::UnitY(), Vec3d( 0, 0, 1 ) },
		{ ToTransformMatrix( Vec3d( 0, 0, 1 ) ), 1 },
		{ -M_PI, M_PI }
		);

	return chain;
}

// ------------------------------------------------------------

Mat4d createPrismaticBaseHome()
{
	return ToTransformMatrix( Vec3d( 1, 0, 1 ) );
}

// ------------------------------------------------------------

Model::KinematicTopology createPrismaticBaseTopology( const Mat4d& home )
{
	Model::KinematicTopology topology;

	Model::PrismaticBaseJointGroup base_group( home );

	topology.Add( base_group );

	return topology;
}

// ------------------------------------------------------------

Model::Skeleton createPrismaticBaseSkeleton(
	const Model::JointChain& chain,
	const Mat4d& home )
{
	std::vector< Model::ArticulationConstPtr > articulations;
	std::vector< Model::BoneConstPtr > bones;

	articulations.emplace_back(
		std::make_shared< const Model::Articulation >(
			Model::ArticulationType::Prismatic,
			std::vector< Model::JointConstPtr > { chain.GetActiveJoint( 0 ) },
			chain.GetActiveJoint( 0 )->Origin()
			)
		);

	articulations.emplace_back(
		std::make_shared< const Model::Articulation >(
			Model::ArticulationType::Revolute,
			std::vector< Model::JointConstPtr > { chain.GetActiveJoint( 1 ) },
			chain.GetActiveJoint( 1 )->Origin()
			)
		);

	bones.emplace_back(
		std::make_shared< const Model::Bone >(
			chain.GetActiveJoint( 0 )->Origin(),
			chain.GetActiveJoint( 1 )->Origin() - chain.GetActiveJoint( 0 )->Origin() )
		);

	bones.emplace_back(
		std::make_shared< const Model::Bone >(
			chain.GetActiveJoint( 1 )->Origin(),
			Translation( home ) - chain.GetActiveJoint( 1 )->Origin() )
		);

	double total_length = 0.0;
	for ( int i = 0; i < bones.size(); i++ )
		total_length += bones[i]->Length();

	return Model::Skeleton( articulations, bones, total_length, chain.GetJointCount() );
}

// ------------------------------------------------------------

Model::KinematicModelConstPtr createPrismaticBaseRobot()
{
	auto chain = createPrismaticBaseJointChain();
	auto home = createPrismaticBaseHome();
	auto topology = createPrismaticBaseTopology( home );
	auto skeleton = std::make_shared< const Model::Skeleton >( createPrismaticBaseSkeleton( *chain, home ) );
	auto reachable_space = createSkeletonReachableSpacePtr( *skeleton );
	return std::make_shared< const Model::KinematicModel >(
		std::move( chain ),
		home,
		topology,
		skeleton,
		std::move( reachable_space ) );
}

// ------------------------------------------------------------

Model::KinematicModelConstPtr Data::GetPrismaticBaseRobot()
{
	if ( !PrismaticBase_robot_ )
	{
		PrismaticBase_robot_ = createPrismaticBaseRobot();
	}
	return PrismaticBase_robot_;
}

// ------------------------------------------------------------
// Planar 2R Robot
// ------------------------------------------------------------

std::unique_ptr< const Model::JointChain > createPlanar2RJointChain()
{
	auto chain = std::make_unique< Model::JointChain >( 2 );

	chain->Add(
		{ Vec3d::UnitX(), Vec3d::Zero() },
		{ Mat4d::Identity(), 0.5 },
		{ -M_PI / 2, M_PI / 2 }
		);

	chain->Add(
		{ Vec3d::UnitX(), Vec3d( 0, 0, 0.5 ) },
		{ ToTransformMatrix( Vec3d( 0, 0, 0.5 ) ), 0.5 },
		{ -M_PI / 2, M_PI / 2 }
		);

	return chain;
}

// ------------------------------------------------------------

Mat4d createPlanar2RHome()
{
	return ToTransformMatrix( Vec3d( 0, 0, 1 ) );
}

// ------------------------------------------------------------

Model::KinematicTopology createPlanar2RTopology( const Mat4d& home )
{
	Model::KinematicTopology topology;

	Model::PlanarNRJointGroup planar_group( 0, 2, home );

	topology.Add( planar_group );

	return topology;
}

// ------------------------------------------------------------

Model::Skeleton createPlanar2RSkeleton(
	const Model::JointChain& chain,
	const Mat4d& home )
{
	std::vector< Model::ArticulationConstPtr > articulations;
	std::vector< Model::BoneConstPtr > bones;

	articulations.emplace_back(
		std::make_shared< const Model::Articulation >(
			Model::ArticulationType::Revolute,
			std::vector< Model::JointConstPtr > { chain.GetActiveJoint( 0 ) },
			chain.GetActiveJoint( 0 )->Origin()
			)
		);

	articulations.emplace_back(
		std::make_shared< const Model::Articulation >(
			Model::ArticulationType::Revolute,
			std::vector< Model::JointConstPtr > { chain.GetActiveJoint( 1 ) },
			chain.GetActiveJoint( 1 )->Origin()
			)
		);

	bones.emplace_back(
		std::make_shared< const Model::Bone >(
			chain.GetActiveJoint( 0 )->Origin(),
			chain.GetActiveJoint( 1 )->Origin() - chain.GetActiveJoint( 0 )->Origin() )
		);

	bones.emplace_back(
		std::make_shared< const Model::Bone >(
			chain.GetActiveJoint( 1 )->Origin(),
			Translation( home ) - chain.GetActiveJoint( 1 )->Origin() )
		);

	double total_length = 0.0;
	for ( int i = 0; i < bones.size(); i++ )
		total_length += bones[i]->Length();

	return Model::Skeleton( articulations, bones, total_length, chain.GetJointCount() );
}

// ------------------------------------------------------------

Model::KinematicModelConstPtr createPlanar2RRobot()
{
	auto chain = createPlanar2RJointChain();
	auto home = createPlanar2RHome();
	auto topology = createPlanar2RTopology( home );
	auto skeleton = std::make_shared< const Model::Skeleton >( createPlanar2RSkeleton( *chain, home ) );
	auto reachable_space = createSkeletonReachableSpacePtr( *skeleton );
	return std::make_shared< const Model::KinematicModel >(
		std::move( chain ),
		home,
		topology,
		std::move( skeleton ),
		std::move( reachable_space ) );
}

// ------------------------------------------------------------

Model::KinematicModelConstPtr Data::GetPlanar2RRobot()
{
	if ( !Planar2R_robot_ )
	{
		Planar2R_robot_ = createPlanar2RRobot();
	}
	return Planar2R_robot_;
}

// ------------------------------------------------------------
// Planar 3R Robot
// ------------------------------------------------------------

std::unique_ptr< const Model::JointChain > createPlanar3RJointChain()
{
	auto chain = std::make_unique< Model::JointChain >( 3 );

	chain->Add(
		{ Vec3d::UnitX(), Vec3d::Zero() },
		{ Mat4d::Identity(), 0.5 },
		{ -M_PI / 2, M_PI / 2 }
		);

	chain->Add(
		{ Vec3d::UnitX(), Vec3d( 0, 0, 0.5 ) },
		{ ToTransformMatrix( Vec3d( 0, 0, 0.5 ) ), 0.5 },
		{ -M_PI / 2, M_PI / 2 }
		);

	chain->Add(
		{ Vec3d::UnitX(), Vec3d( 0, 0, 1 ) },
		{ ToTransformMatrix( Vec3d( 0, 0, 1.0 ) ), 0.1 },
		{ -M_PI / 2, M_PI / 2 }
		);

	return chain;
}

// ------------------------------------------------------------

Mat4d createPlanar3RHome()
{
	return ToTransformMatrix( Vec3d( 0.1, 0, 1 ) );
}

// ------------------------------------------------------------

Model::KinematicTopology createPlanar3RTopology( const Mat4d& home )
{
	Model::KinematicTopology topology;

	Model::PlanarNRJointGroup planar_group( 0, 3, Mat4d::Identity() );

	topology.Add( planar_group );

	return topology;
}

// ------------------------------------------------------------

Model::Skeleton createPlanar3RSkeleton(
	const Model::JointChain& chain,
	const Mat4d& home )
{
	std::vector< Model::ArticulationConstPtr > articulations;
	std::vector< Model::BoneConstPtr > bones;

	articulations.emplace_back(
		std::make_shared< const Model::Articulation >(
			Model::ArticulationType::Revolute,
			std::vector< Model::JointConstPtr > { chain.GetActiveJoint( 0 ) },
			chain.GetActiveJoint( 0 )->Origin()
			)
		);

	articulations.emplace_back(
		std::make_shared< const Model::Articulation >(
			Model::ArticulationType::Revolute,
			std::vector< Model::JointConstPtr > { chain.GetActiveJoint( 1 ) },
			chain.GetActiveJoint( 1 )->Origin()
			)
		);

	articulations.emplace_back(
		std::make_shared< const Model::Articulation >(
			Model::ArticulationType::Revolute,
			std::vector< Model::JointConstPtr > { chain.GetActiveJoint( 2 ) },
			Translation( home )
			)
		);

	bones.emplace_back(
		std::make_shared< const Model::Bone >(
			chain.GetActiveJoint( 0 )->Origin(),
			chain.GetActiveJoint( 1 )->Origin() - chain.GetActiveJoint( 0 )->Origin() )
		);

	bones.emplace_back(
		std::make_shared< const Model::Bone >(
			chain.GetActiveJoint( 1 )->Origin(),
			Translation( home ) - chain.GetActiveJoint( 1 )->Origin() )
		);

	double total_length = 0.0;
	for ( int i = 0; i < bones.size(); i++ )
		total_length += bones[i]->Length();

	return Model::Skeleton( articulations, bones, total_length, chain.GetJointCount() );
}

// ------------------------------------------------------------

Model::KinematicModelConstPtr createPlanar3RRobot()
{
	auto chain = createPlanar3RJointChain();
	auto home = createPlanar3RHome();
	auto topology = createPlanar3RTopology( home );
	auto skeleton = std::make_shared< const Model::Skeleton >( createPlanar3RSkeleton( *chain, home ) );
	auto reachable_space = createSkeletonReachableSpacePtr( *skeleton );
	return std::make_shared< const Model::KinematicModel >(
		std::move( chain ),
		home,
		topology,
		skeleton,
		std::move( reachable_space ) );
}

// ------------------------------------------------------------

Model::KinematicModelConstPtr Data::GetPlanar3RRobot()
{
	if ( !Planar3R_robot_ )
	{
		Planar3R_robot_ = createPlanar3RRobot();
	}
	return Planar3R_robot_;
}

// ------------------------------------------------------------
// 1 Dof Wrist Robot
// ------------------------------------------------------------

std::unique_ptr< const Model::JointChain > createWrist1RJointChain()
{
	auto chain = std::make_unique< Model::JointChain >( 1 );

	chain->Add(
		{ Vec3d::UnitX(), Vec3d( 0, 0, 1 ) },
		{ ToTransformMatrix( Vec3d( 0, 0, 1 ) ), 0.0 },
		{ -M_PI / 2, M_PI / 2 }
		);

	return chain;
}

// ------------------------------------------------------------

Mat4d createWrist1RHome()
{
	return ToTransformMatrix( Vec3d( 0.0, 0, 1 ) );
}

// ------------------------------------------------------------

Model::KinematicTopology createWrist1RTopology( const Mat4d& home )
{
	Model::KinematicTopology topology;

	Model::WristJointGroup wrist_group( 0, 1, home );

	topology.Add( wrist_group );

	return topology;
}

// ------------------------------------------------------------

Model::Skeleton createWrist1RSkeleton(
	const Model::JointChain& chain,
	const Mat4d& home )
{
	std::vector< Model::ArticulationConstPtr > articulations;
	std::vector< Model::BoneConstPtr > bones;

	articulations.emplace_back(
		std::make_shared< const Model::Articulation >(
			Model::ArticulationType::Revolute,
			std::vector< Model::JointConstPtr > { chain.GetActiveJoint( 0 ) },
			chain.GetActiveJoint( 0 )->Origin()
			)
		);

	double total_length = 0.0;
	for ( int i = 0; i < bones.size(); i++ )
		total_length += bones[i]->Length();

	return Model::Skeleton( articulations, bones, total_length, chain.GetJointCount() );
}

// ------------------------------------------------------------

Model::KinematicModelConstPtr createWrist1RRobot()
{
	auto chain = createWrist1RJointChain();
	auto home = createWrist1RHome();
	auto topology = createWrist1RTopology( home );
	auto skeleton = std::make_shared< const Model::Skeleton >( createWrist1RSkeleton( *chain, home ) );
	auto reachable_space = createSkeletonReachableSpacePtr( *skeleton );
	return std::make_shared< const Model::KinematicModel >(
		std::move( chain ),
		home,
		topology,
		skeleton,
		std::move( reachable_space ) );
}

// ------------------------------------------------------------

Model::KinematicModelConstPtr Data::GetWrist1RRobot()
{
	if ( !Wrist1R_robot_ )
	{
		Wrist1R_robot_ = createWrist1RRobot();
	}
	return Wrist1R_robot_;
}

// ------------------------------------------------------------
// 2 Dof Wrist Robot
// ------------------------------------------------------------

std::unique_ptr< const Model::JointChain > createWrist2RJointChain()
{
	auto chain = std::make_unique< Model::JointChain >( 2 );

	chain->Add(
		{ Vec3d::UnitX(), Vec3d( 0, 0, 1 ) },
		{ ToTransformMatrix( Vec3d( 0, 0, 1 ) ), 0.0 },
		{ -M_PI / 2, M_PI / 2 }
		);

	chain->Add(
		{ Vec3d::UnitY(), Vec3d( 0, 0, 1 ) },
		{ ToTransformMatrix( Vec3d( 0, 0, 1 ) ), 0.0 },
		{ -M_PI / 2, M_PI / 2 }
		);

	return chain;
}

// ------------------------------------------------------------

Mat4d createWrist2RHome()
{
	return ToTransformMatrix( Vec3d( 0.0, 0, 1 ) );
}

// ------------------------------------------------------------

Model::KinematicTopology createWrist2RTopology( const Mat4d& home )
{
	Model::KinematicTopology topology;

	Model::WristJointGroup wrist_group( 0, 2, home );

	topology.Add( wrist_group );

	return topology;
}

// ------------------------------------------------------------

Model::Skeleton createWrist2RSkeleton(
	const Model::JointChain& chain,
	const Mat4d& home )
{
	std::vector< Model::ArticulationConstPtr > articulations;
	std::vector< Model::BoneConstPtr > bones;

	articulations.emplace_back(
		std::make_shared< const Model::Articulation >(
			Model::ArticulationType::Universal,
			std::vector< Model::JointConstPtr > { chain.GetActiveJoint( 0 ), chain.GetActiveJoint( 1 ) },
			chain.GetActiveJoint( 1 )->Origin()
			)
		);

	double total_length = 0.0;
	for ( int i = 0; i < bones.size(); i++ )
		total_length += bones[i]->Length();

	return Model::Skeleton( articulations, bones, total_length, chain.GetJointCount() );
}

// ------------------------------------------------------------

Model::KinematicModelConstPtr createWrist2RRobot()
{
	auto chain = createWrist2RJointChain();
	auto home = createWrist2RHome();
	auto topology = createWrist2RTopology( home );
	auto skeleton = std::make_shared< const Model::Skeleton >( createWrist2RSkeleton( *chain, home ) );
	auto reachable_space = createSkeletonReachableSpacePtr( *skeleton );
	return std::make_shared< const Model::KinematicModel >(
		std::move( chain ),
		home,
		topology,
		skeleton,
		std::move( reachable_space ) );
}

// ------------------------------------------------------------

Model::KinematicModelConstPtr Data::GetWrist2RRobot()
{
	if ( !Wrist2R_robot_ )
	{
		Wrist2R_robot_ = createWrist2RRobot();
	}
	return Wrist2R_robot_;
}


// ------------------------------------------------------------
// Spherical Wrist Robot
// ------------------------------------------------------------

std::unique_ptr< const Model::JointChain > createSphericalWristJointChain()
{
	auto chain = std::make_unique< Model::JointChain >( 3 );

	chain->Add(
		{ Vec3d::UnitX(), Vec3d( 0, 0, 1 ) },
		{ ToTransformMatrix( Vec3d( 0, 0, 1 ) ), 0.0 },
		{ -M_PI / 2, M_PI / 2 }
		);

	chain->Add(
		{ Vec3d::UnitY(), Vec3d( 0, 0, 1 ) },
		{ ToTransformMatrix( Vec3d( 0, 0, 1 ) ), 0.0 },
		{ -M_PI / 2, M_PI / 2 }
		);

	chain->Add(
		{ Vec3d::UnitZ(), Vec3d( 0, 0, 1 ) },
		{ ToTransformMatrix( Vec3d( 0, 0, 1 ) ), 0.1 },
		{ -M_PI / 2, M_PI / 2 }
		);

	return chain;
}

// ------------------------------------------------------------

Mat4d createSphericalWristHome()
{
	return ToTransformMatrix( Vec3d( 0.1, 0, 1 ) );
}

// ------------------------------------------------------------

Model::KinematicTopology createSphericalWristTopology( const Mat4d& home )
{
	Model::KinematicTopology topology;

	Model::WristJointGroup wrist_group( 0, 3, home );

	topology.Add( wrist_group );

	return topology;
}

// ------------------------------------------------------------

Model::Skeleton createSphericalWristSkeleton(
	const Model::JointChain& chain,
	const Mat4d& home )
{
	std::vector< Model::ArticulationConstPtr > articulations;
	std::vector< Model::BoneConstPtr > bones;

	articulations.emplace_back(
		std::make_shared< const Model::Articulation >(
			Model::ArticulationType::Spherical,
			std::vector< Model::JointConstPtr > { chain.GetActiveJoint( 0 ), chain.GetActiveJoint( 1 ), chain.GetActiveJoint( 2 ) },
			chain.GetActiveJoint( 1 )->Origin()
			)
		);

	bones.emplace_back(
		std::make_shared< const Model::Bone >(
			chain.GetActiveJoint( 0 )->Origin(),
			Translation( home ) - chain.GetActiveJoint( 0 )->Origin() )
		);

	double total_length = 0.0;
	for ( int i = 0; i < bones.size(); i++ )
		total_length += bones[i]->Length();

	return Model::Skeleton( articulations, bones, total_length, chain.GetJointCount() );
}

// ------------------------------------------------------------

Model::KinematicModelConstPtr createSphericalWristRobot()
{
	auto chain = createSphericalWristJointChain();
	auto home = createSphericalWristHome();
	auto topology = createSphericalWristTopology( home );
	auto skeleton = std::make_shared< const Model::Skeleton >( createSphericalWristSkeleton( *chain, home ) );
	auto reachable_space = createSkeletonReachableSpacePtr( *skeleton );
	return std::make_shared< const Model::KinematicModel >(
		std::move( chain ),
		home,
		topology,
		skeleton,
		std::move( reachable_space ) );
}

// ------------------------------------------------------------

Model::KinematicModelConstPtr Data::GetSphericalWristRobot()
{
	if ( !SphericalWrist_robot_ )
	{
		SphericalWrist_robot_ = createSphericalWristRobot();
	}
	return SphericalWrist_robot_;
}

// ------------------------------------------------------------
// Revolute Base - Planar 2R - 2DOFs Wrist / 5DOFs Robot
// ------------------------------------------------------------

std::unique_ptr< const Model::JointChain > createRevolute_Planar2R_Wrist2R_5DOFsJointChain()
{
	// Create a joint chain with 5 joints: 1 base joint + 2 planar joints + 2 wrist joints
	auto chain = std::make_unique< Model::JointChain >( 5 );

	// Base joint (revolute around Z-axis)
	Vec3d origin      = Vec3d( 0, 0, 0.0 );
	Vec3d next_origin = Vec3d( 0, 0, 0.5 );
	Vec3d axis = Vec3d::UnitZ();
	chain->Add(
		Model::Twist( axis, origin ),
		Model::Link( ToTransformMatrix( origin ), ToTransformMatrix( next_origin ) ),
		Model::Limits( -M_PI, M_PI )
		);

	// Planar joints (2 revolute joints)
	origin = next_origin;
	next_origin = Vec3d( 0, 0, 1.0 );
	axis = Vec3d::UnitY();
	chain->Add(
		Model::Twist( axis, origin ),
		Model::Link( ToTransformMatrix( origin ), ToTransformMatrix( next_origin ) ),
		Model::Limits( -M_PI / 2, M_PI / 2 )
		);

	origin = next_origin;
	next_origin = Vec3d( 0, 0, 1.5 );
	axis = Vec3d::UnitY();
	chain->Add(
		Model::Twist( axis, origin ),
		Model::Link( ToTransformMatrix( origin ), ToTransformMatrix( next_origin ) ),
		Model::Limits( -M_PI / 2, M_PI / 2 )
		);

	// Wrist joints (2 revolute joints - 2R wrist)
	origin = next_origin;
	next_origin = origin;
	next_origin.z() += 0.1;
	axis = Vec3d::UnitX();
	chain->Add(
		Model::Twist( axis, origin ),
		Model::Link( ToTransformMatrix( origin ), ToTransformMatrix( next_origin ) ),
		Model::Limits( -M_PI, M_PI )
		);

	origin = next_origin;
	axis = Vec3d::UnitZ();
	chain->Add(
		Model::Twist( axis, origin ),
		Model::Link( ToTransformMatrix( origin ), 0.1 ),
		Model::Limits( -M_PI, M_PI ) );

	return chain;
}

// ------------------------------------------------------------

Mat4d createRevolute_Planar2R_Wrist2R_5DOFsHome()
{
	return ToTransformMatrix( Vec3d( 0.1, 0, 1.4 ) );
}

// ------------------------------------------------------------

Model::KinematicTopology createRevolute_Planar2R_Wrist2R_5DOFsTopology( const Mat4d& home )
{
	Model::KinematicTopology topology;

	Mat4d wrist_center = ToTransformMatrix( Vec3d( 0.1, 0, 1.5 ) );

	Model::RevoluteBaseJointGroup base_group( wrist_center );
	Model::PlanarNRJointGroup planar_group( 0, 2, wrist_center );
	Model::WristJointGroup wrist_group( 2, 2, home * Inverse( wrist_center ) );

	topology.Add( base_group );
	topology.Add( planar_group );
	topology.Add( wrist_group );

	return topology;
}

// ------------------------------------------------------------

Model::Skeleton createRevolute_Planar2R_Wrist2R_5DOFsSkeleton(
	const Model::JointChain& chain,
	const Mat4d& home )
{
	std::vector< Model::ArticulationConstPtr > articulations;
	std::vector< Model::BoneConstPtr > bones;

	articulations.emplace_back(
		std::make_shared< const Model::Articulation >(
			Model::ArticulationType::Universal,
			std::vector< Model::JointConstPtr > { chain.GetActiveJoint( 0 ), chain.GetActiveJoint( 1 ) },
			chain.GetActiveJoint( 1 )->Origin()
			)
		);

	articulations.emplace_back(
		std::make_shared< const Model::Articulation >(
			Model::ArticulationType::Revolute,
			std::vector< Model::JointConstPtr > { chain.GetActiveJoint( 2 ) },
			chain.GetActiveJoint( 2 )->Origin()
			)
		);

	articulations.emplace_back(
		std::make_shared< const Model::Articulation >(
			Model::ArticulationType::Universal,
			std::vector< Model::JointConstPtr > { chain.GetActiveJoint( 3 ), chain.GetActiveJoint(  4 ) },
			Vec3d{ 0, 0, 1.5 }
			)
		);

	bones.emplace_back(
		std::make_shared< const Model::Bone >(
			articulations[0]->Center(),
			articulations[1]->Center() - articulations[0]->Center() )
		);

	bones.emplace_back(
		std::make_shared< const Model::Bone >(
			articulations[1]->Center(),
			articulations[2]->Center() - articulations[1]->Center() )
		);

	bones.emplace_back(
		std::make_shared< const Model::Bone >(
			articulations[2]->Center(),
			Translation( home ) - articulations[2]->Center() )
		);

	double total_length = articulations[0]->Center().norm();
	for ( int i = 0; i < bones.size(); i++ )
		total_length += bones[i]->Length();

	return Model::Skeleton( articulations, bones, total_length, chain.GetJointCount() );
}

// ------------------------------------------------------------

Model::KinematicModelConstPtr createRevolute_Planar2R_Wrist2R_5DOFsRobot()
{
	auto chain = createRevolute_Planar2R_Wrist2R_5DOFsJointChain();
	auto home = createRevolute_Planar2R_Wrist2R_5DOFsHome();
	auto topology = createRevolute_Planar2R_Wrist2R_5DOFsTopology( home );
	auto skeleton = std::make_shared< const Model::Skeleton >( createRevolute_Planar2R_Wrist2R_5DOFsSkeleton( *chain, home ) );
	auto reachable_space = createSkeletonReachableSpacePtr( *skeleton );
	return std::make_shared< const Model::KinematicModel >(
		std::move( chain ),
		home,
		topology,
		skeleton,
		std::move( reachable_space ) );
}

// ------------------------------------------------------------

Model::KinematicModelConstPtr Data::GetRevolute_Planar2R_Wrist2R_5DOFsRobot()
{
	if ( !Revolute_Planar2R_Wrist2R_5DOFs_robot_ )
	{
		Revolute_Planar2R_Wrist2R_5DOFs_robot_ = createRevolute_Planar2R_Wrist2R_5DOFsRobot();
	}
	return Revolute_Planar2R_Wrist2R_5DOFs_robot_;
}

// ------------------------------------------------------------
// Revolute Base - Planar 2R - Spherical Wrist / 6DOFs Robot
// ------------------------------------------------------------

std::unique_ptr< const Model::JointChain > createRevolute_Planar2R_SphericalWrist_6DOFsJointChain()
{
	// Create a joint chain with 6 joints: 1 base joint + 2 numeric joints + 3 wrist joints
	auto chain = std::make_unique< Model::JointChain >( 6 );

	// Base joint (revolute around Z-axis)
	Vec3d origin      = Vec3d( 0, 0, 0.0 );
	Vec3d next_origin = Vec3d( 0, 0, 0.5 );
	Vec3d axis = Vec3d::UnitZ();
	chain->Add(
		Model::Twist( axis, origin ),
		Model::Link( ToTransformMatrix( origin ), ToTransformMatrix( next_origin ) ),
		Model::Limits( -M_PI, M_PI )
		);

	// Planar joints (2 revolute joints)
	origin = next_origin;
	next_origin = Vec3d( 0, 0, 1.0 );
	axis = Vec3d::UnitY();
	chain->Add(
		Model::Twist( axis, origin ),
		Model::Link( ToTransformMatrix( origin ), ToTransformMatrix( next_origin ) ),
		Model::Limits( -M_PI / 2, M_PI / 2 )
		);

	origin = next_origin;
	next_origin = Vec3d( 0, 0, 1.5 );
	axis = Vec3d::UnitY();
	chain->Add(
		Model::Twist( axis, origin ),
		Model::Link( ToTransformMatrix( origin ), ToTransformMatrix( next_origin ) ),
		Model::Limits( -M_PI / 2, M_PI / 2 )
		);

	// Wrist joints (3 revolute joints - spherical wrist)
	origin = next_origin;
	axis = Vec3d::UnitX();
	chain->Add(
		Model::Twist( axis, origin ),
		Model::Link( ToTransformMatrix( origin ), ToTransformMatrix( next_origin ) ),
		Model::Limits( -M_PI, M_PI )
		);

	axis = Vec3d::UnitY();
	chain->Add(
		Model::Twist( axis, origin ),
		Model::Link( ToTransformMatrix( origin ), ToTransformMatrix( next_origin ) ),
		Model::Limits( -M_PI, M_PI )
		);

	axis = Vec3d::UnitZ();
	chain->Add(
		Model::Twist( axis, origin ),
		Model::Link( ToTransformMatrix( origin ), ToTransformMatrix( next_origin ) ),
		Model::Limits( -M_PI, M_PI )
		);

	return chain;
}

// ------------------------------------------------------------

Mat4d createRevolute_Planar2R_SphericalWrist_6DOFsHome()
{
	return ToTransformMatrix( Vec3d( 0.0, 0, 1.5 ) );
}

// ------------------------------------------------------------

Model::KinematicTopology createRevolute_Planar2R_SphericalWrist_6DOFsTopology( const Mat4d& home )
{
	Model::KinematicTopology topology;

	Mat4d wrist_center = ToTransformMatrix( Vec3d( 0., 0., 1.5 ) );

	Model::RevoluteBaseJointGroup base_group( wrist_center );
	Model::PlanarNRJointGroup planar_group( 0, 2, wrist_center );
	Model::WristJointGroup wrist_group( 2, 3, home * Inverse( wrist_center ) );

	topology.Add( base_group );
	topology.Add( planar_group );
	topology.Add( wrist_group );

	return topology;
}

// ------------------------------------------------------------

Model::Skeleton createRevolute_Planar2R_SphericalWrist_6DOFsSkeleton(
	const Model::JointChain& chain,
	const Mat4d& home )
{
	std::vector< Model::ArticulationConstPtr > articulations;
	std::vector< Model::BoneConstPtr > bones;

	articulations.emplace_back(
		std::make_shared< const Model::Articulation >(
			Model::ArticulationType::Universal,
			std::vector< Model::JointConstPtr > { chain.GetActiveJoint( 0 ), chain.GetActiveJoint( 1 ) },
			chain.GetActiveJoint( 1 )->Origin()
			)
		);

	articulations.emplace_back(
		std::make_shared< const Model::Articulation >(
			Model::ArticulationType::Revolute,
			std::vector< Model::JointConstPtr > { chain.GetActiveJoint( 2 ) },
			chain.GetActiveJoint( 2 )->Origin()
			)
		);

	articulations.emplace_back(
		std::make_shared< const Model::Articulation >(
			Model::ArticulationType::Spherical,
			std::vector< Model::JointConstPtr > { chain.GetActiveJoint( 3 ), chain.GetActiveJoint(  4 ), chain.GetActiveJoint(  5 ) },
			chain.GetActiveJoint( 5 )->Origin()
			)
		);

	bones.emplace_back(
		std::make_shared< const Model::Bone >(
			articulations[0]->Center(),
			articulations[1]->Center() - articulations[0]->Center() )
		);

	bones.emplace_back(
		std::make_shared< const Model::Bone >(
			articulations[1]->Center(),
			articulations[2]->Center() - articulations[1]->Center() )
		);

	double total_length = articulations[0]->Center().norm();
	for ( int i = 0; i < bones.size(); i++ )
		total_length += bones[i]->Length();

	return Model::Skeleton( articulations, bones, total_length, chain.GetJointCount() );
}

// ------------------------------------------------------------

Model::KinematicModelConstPtr createRevolute_Planar2R_SphericalWrist_6DOFsRobot()
{
	auto chain = createRevolute_Planar2R_SphericalWrist_6DOFsJointChain();
	auto home = createRevolute_Planar2R_SphericalWrist_6DOFsHome();
	auto topology = createRevolute_Planar2R_SphericalWrist_6DOFsTopology( home );
	auto skeleton = std::make_shared< const Model::Skeleton >( createRevolute_Planar2R_SphericalWrist_6DOFsSkeleton( *chain, home ) );
	auto reachable_space = createSkeletonReachableSpacePtr( *skeleton );
	return std::make_shared< const Model::KinematicModel >(
		std::move( chain ),
		home,
		topology,
		skeleton,
		std::move( reachable_space ) );
}

// ------------------------------------------------------------

Model::KinematicModelConstPtr Data::GetRevolute_Planar2R_SphericalWrist_6DOFsRobot()
{
	if ( !Revolute_Planar2R_SphericalWrist_6DOFs_robot_ )
	{
		Revolute_Planar2R_SphericalWrist_6DOFs_robot_ = createRevolute_Planar2R_SphericalWrist_6DOFsRobot();
	}
	return Revolute_Planar2R_SphericalWrist_6DOFs_robot_;
}

// ------------------------------------------------------------
// UR Like / 6DOFs Robot
// ------------------------------------------------------------

std::unique_ptr< const Model::JointChain > createURLike_6DOFsJointChain()
{
	// Create a joint chain with 6 joints: 1 base joint + 2 numeric joints + 3 wrist joints
	auto chain = std::make_unique< Model::JointChain >( 6 );

	// Base Joint
	Vec3d origin      = Vec3d( 0, 0, 0.0 );
	Vec3d next_origin = Vec3d( 0, 0.2, 0.2 );
	Vec3d axis = Vec3d::UnitZ();
	chain->Add(
		Model::Twist( axis, origin ),
		Model::Link( ToTransformMatrix( origin ), ToTransformMatrix( next_origin ) ),
		Model::Limits( -M_PI, M_PI )
		);

	// Planar Joints 3 Y-axis revolute joints
	origin = next_origin;
	next_origin = Vec3d( 0, 0.2, 0.6 );
	axis = Vec3d::UnitY();
	chain->Add(
		Model::Twist( axis, origin ),
		Model::Link( ToTransformMatrix( origin ), ToTransformMatrix( next_origin ) ),
		Model::Limits( -M_PI / 2, M_PI / 2 )
		);

	origin = next_origin;
	next_origin = Vec3d( 0.4, 0, 0.6 );
	axis = Vec3d::UnitY();
	chain->Add(
		Model::Twist( axis, origin ),
		Model::Link( ToTransformMatrix( origin ), ToTransformMatrix( next_origin ) ),
		Model::Limits( -M_PI / 2, M_PI / 2 )
		);

	origin = next_origin;
	next_origin = Vec3d( 0.4, 0.2, 0.6 );
	axis = Vec3d::UnitY();
	chain->Add(
		Model::Twist( axis, origin ),
		Model::Link( ToTransformMatrix( origin ), ToTransformMatrix( next_origin ) ),
		Model::Limits( -M_PI, M_PI )
		);

	// Wrist 2 revolute joints
	origin = next_origin;
	next_origin = Vec3d( 0.5, 0.2, 0.6 );
	axis = Vec3d::UnitX();
	chain->Add(
		Model::Twist( axis, origin ),
		Model::Link( ToTransformMatrix( origin ), ToTransformMatrix( next_origin ) ),
		Model::Limits( -M_PI, M_PI )
		);

	origin = next_origin;
	axis = Vec3d::UnitZ();
	chain->Add(
		Model::Twist( axis, origin ),
		Model::Link( ToTransformMatrix( origin ), 0 ),
		Model::Limits( -M_PI, M_PI )
		);

	return chain;
}

// ------------------------------------------------------------

Mat4d createURLike_6DOFsHome()
{
	return ToTransformMatrix( Vec3d( 0.5, 0.2, 0.5 ) );
}

// ------------------------------------------------------------

Model::KinematicTopology createURLike_6DOFsTopology( const Mat4d& home )
{
	Model::KinematicTopology topology;

	Mat4d wrist_center = ToTransformMatrix( Vec3d( 0.5, 0.2, 0.6 ) );

	Model::RevoluteBaseJointGroup base_group( wrist_center );
	Model::PlanarNRJointGroup planar_group( 0, 2, wrist_center );
	Model::WristJointGroup wrist_group( 3, 2, home * Inverse( wrist_center ) );

	topology.Add( base_group );
	topology.Add( planar_group );
	topology.Add( wrist_group );

	return topology;
}

// ------------------------------------------------------------

Model::Skeleton createURLike_6DOFsSkeleton(
	const Model::JointChain& chain,
	const Mat4d& home )
{
	std::vector< Model::ArticulationConstPtr > articulations;
	std::vector< Model::BoneConstPtr > bones;

	articulations.emplace_back(
		std::make_shared< const Model::Articulation >(
			Model::ArticulationType::Universal,
			std::vector< Model::JointConstPtr > { chain.GetActiveJoint( 0 ), chain.GetActiveJoint( 1 ) },
			Vec3d{ 0, 0, 0.2 }
			)
		);

	articulations.emplace_back(
		std::make_shared< const Model::Articulation >(
			Model::ArticulationType::Revolute,
			std::vector< Model::JointConstPtr > { chain.GetActiveJoint( 2 ) },
			chain.GetActiveJoint( 2 )->Origin()
			)
		);

	articulations.emplace_back(
		std::make_shared< const Model::Articulation >(
			Model::ArticulationType::Revolute,
			std::vector< Model::JointConstPtr > { chain.GetActiveJoint( 3 ) },
			chain.GetActiveJoint( 3 )->Origin()
			)
		);

	articulations.emplace_back(
		std::make_shared< const Model::Articulation >(
			Model::ArticulationType::Universal,
			std::vector< Model::JointConstPtr > { chain.GetActiveJoint(  4 ), chain.GetActiveJoint(  5 ) },
			chain.GetActiveJoint( 5 )->Origin()
			)
		);

	bones.emplace_back(
		std::make_shared< const Model::Bone >(
			articulations[0]->Center(),
			articulations[1]->Center() - articulations[0]->Center() )
		);

	bones.emplace_back(
		std::make_shared< const Model::Bone >(
			articulations[1]->Center(),
			articulations[2]->Center() - articulations[1]->Center() )
		);

	bones.emplace_back(
		std::make_shared< const Model::Bone >(
			articulations[2]->Center(),
			articulations[3]->Center() - articulations[2]->Center() )
		);

	bones.emplace_back(
		std::make_shared< const Model::Bone >(
			articulations[3]->Center(),
			Translation( home ) - articulations[3]->Center() )
		);


	double total_length = articulations[0]->Center().norm();
	for ( int i = 0; i < bones.size(); i++ )
		total_length += bones[i]->Length();

	return Model::Skeleton( articulations, bones, total_length, chain.GetJointCount() );
}

// ------------------------------------------------------------

Model::KinematicModelConstPtr createURLike_6DOFsRobot()
{
	auto chain = createURLike_6DOFsJointChain();
	auto home = createURLike_6DOFsHome();
	auto topology = createURLike_6DOFsTopology( home );
	auto skeleton = std::make_shared< const Model::Skeleton >( createURLike_6DOFsSkeleton( *chain, home ) );
	auto reachable_space = createSkeletonReachableSpacePtr( *skeleton );
	return std::make_shared< const Model::KinematicModel >(
		std::move( chain ),
		home,
		topology,
		skeleton,
		std::move( reachable_space ) );
}

// ------------------------------------------------------------

Model::KinematicModelConstPtr Data::GetURLikeRobot()
{
	if ( !URLike_robot_ )
	{
		URLike_robot_ = createURLike_6DOFsRobot();
	}
	return URLike_robot_;
}

// ------------------------------------------------------------

}