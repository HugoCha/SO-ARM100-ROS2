#include "RobotModelTestData.hpp"

#include "Global.hpp"

#include "Model/Joint/Joint.hpp"
#include "Model/Joint/JointChain.hpp"
#include "Model/Joint/JointChainBuilder.hpp"
#include "Model/Joint/JointGroup.hpp"
#include "Model/Joint/JointType.hpp"
#include "Model/Joint/Limits.hpp"
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

#include <cctype>
#include <cmath>
#include <Eigen/Dense>
#include <memory>
#include <moveit/robot_model/joint_model.hpp>
#include <moveit/robot_model/link_model.hpp>
#include <moveit/robot_model/prismatic_joint_model.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <optional>
#include <stdexcept>
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
Model::KinematicModelConstPtr LeRobot_robot_ = nullptr;
Model::KinematicModelConstPtr LeRobotWithGripper_robot_ = nullptr;

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

Model::JointChainBuilder& AddBaseLink( Model::JointChainBuilder& builder )
{
	builder.AddParentLink( "base", Mat4d::Identity() );
	return builder;
}

Model::JointChainBuilder& AddRevoluteJointLink(
	Model::JointChainBuilder& builder, 
	int index,
	const Mat4d& joint_home,
	const Vec3d& joint_axis,
	const Mat4d& link_home,
	double min = -M_PI, 
	double max = M_PI )
{
	builder.AddJoint( "r_joint" + std::to_string( index ), 
					  joint_home, 
					  { joint_axis, Translation( joint_home ) }, 
					  { min, max } );

	builder.AddChildLink( "link" + std::to_string( index ), 
						  link_home, 
						  link_home * Inverse( joint_home ) );
	
	return builder;
}

Model::JointChainBuilder& AddPrismaticJointLink(
	Model::JointChainBuilder& builder, 
	int index,
	const Mat4d& joint_home,
	const Vec3d& joint_axis,
	const Mat4d& link_home,
	double min = 0, 
	double max = 1 )
{
	builder.AddJoint( "p_joint" + std::to_string( index ), 
					  joint_home, 
					  { joint_axis }, 
					  { min, max } );

	builder.AddChildLink( "link" + std::to_string( index ), 
						  link_home, 
						  link_home * Inverse( joint_home ) );
	
	return builder;
}

Model::JointChainBuilder& AddJointTipLink(
	Model::JointChainBuilder& builder, 
	int index,
	const Model::JointType type,
	const Mat4d& joint_home,
	const Vec3d& joint_axis,
	const Mat4d& tip_home,
	double min = -M_PI, 
	double max = M_PI )
{
	Model::Twist t;
	if ( type == Model::JointType::FIXED )
		t = Model::Twist();
	else if ( type == Model::JointType::PRISMATIC )
		t = Model::Twist( joint_axis );
	else
		t = Model::Twist( joint_axis, Translation( joint_home ) );

	Model::Limits l = type == Model::JointType::FIXED ? Model::Limits{} : Model::Limits{ min, max };

	std::string name;
	switch ( type )
	{
		case Model::JointType::FIXED:
			name = "f_joint" + std::to_string( index );
			break;
		case Model::JointType::REVOLUTE:
			name = "r_joint" + std::to_string( index );
			break;
		case Model::JointType::PRISMATIC:
			name = "p_joint" + std::to_string( index );
			break;
	}

	builder.AddJoint( name, 
					  joint_home, 
					  t, 
					  l );

	builder.AddChildLink( "link_tip", 
						  tip_home, 
						  tip_home * Inverse( joint_home ) );
	
	return builder;
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
                <limit lower="-1.570796" upper="1.570796" effort="100" velocity="1.0"/>
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
                <limit lower="-1.570796" upper="1.570796" effort="100" velocity="1.0"/>
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

Model::JointChainConstPtr createZYZRevoluteRobotJointChain( const Mat4d& home )
{
	auto builder = Model::JointChainBuilder();
	
	Mat4d origin1 = ToTransformMatrix( Vec3d( 0, 0, 0 ) );
	Mat4d origin2 = ToTransformMatrix( Vec3d( 0.5, 0, 0 ) );
	Mat4d origin3 = ToTransformMatrix( Vec3d( 1.0, 0, 0 ) );
	
	AddBaseLink( builder );
	
	AddRevoluteJointLink( builder, 1, origin1, Vec3d::UnitZ(), origin2 );
	AddRevoluteJointLink( builder, 2, origin2, Vec3d::UnitY(), origin3, -M_PI / 2, M_PI / 2 );
	AddJointTipLink( builder, 3, Model::JointType::REVOLUTE, origin3, 
					 Vec3d::UnitZ(), home, -M_PI / 2, M_PI / 2 );

	return builder.Build();
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
	Mat4d wrist_center = ToTransformMatrix( Vec3d( 1, 0, 0 ) );

	Model::RevoluteBaseJointGroup base_group( wrist_center );
	Model::PlanarNRJointGroup planar_group( 1, 1, wrist_center );
	Model::WristJointGroup wrist_group( 2, 1, home * Inverse( wrist_center ), wrist_center );

	topology.Add( base_group );
	topology.Add( planar_group );
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
	auto home = createZYZRevoluteRobotHome();
	auto joint_chain = createZYZRevoluteRobotJointChain( home );
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

const Model::JointChain* Data::GetZYZRevoluteRobotJointChain()
{
	return GetZYZRevoluteRobot()->GetChain();
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

Model::JointChainConstPtr createRevoluteBaseJointChain( const Mat4d& home )
{
	auto builder = Model::JointChainBuilder();

	Mat4d origin1 = ToTransformMatrix( Vec3d( 0, 0 , 0 ) );
	Mat4d origin2 = ToTransformMatrix( Vec3d( 0, 0 , 1 ) );
	Mat4d origin3 = ToTransformMatrix( Vec3d( 1, 0 , 1 ) );

	AddBaseLink( builder );
	AddRevoluteJointLink( builder, 1, origin1, Vec3d::UnitZ(), origin2 );
	AddRevoluteJointLink( builder, 2, origin2, Vec3d::UnitY(), origin3 );
	AddJointTipLink( builder, 3, Model::JointType::REVOLUTE, origin3, Vec3d::UnitY(), home );

	return builder.Build();
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

	Mat4d wrist_center = ToTransformMatrix( Vec3d( 1, 0, 1 ) );

	Model::RevoluteBaseJointGroup base_group( wrist_center );

	Model::PlanarNRJointGroup planar_group( 1, 1, wrist_center );

	Model::WristJointGroup wrist_group(
		2,
		1,
		ToTransformMatrix( Vec3d( 0, 0, -0.1 ) ),
		wrist_center  );

	topology.Add( base_group );
	topology.Add( planar_group );
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
			chain.GetActiveJoint( 2 )->Origin()
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
			Translation( home ) - articulations[1]->Center() )
		);

	double total_length = articulations[0]->Center().norm();
	for ( int i = 0; i < bones.size(); i++ )
		total_length += bones[i]->Length();

	return Model::Skeleton( articulations, bones, total_length, chain.GetJointCount() );
}

// ------------------------------------------------------------

Model::KinematicModelConstPtr createRevoluteBaseRobot()
{
	auto home = createRevoluteBaseHome();
	auto chain = createRevoluteBaseJointChain( home );
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

Model::JointChainConstPtr createPrismaticBaseJointChain( const Mat4d& home )
{
	auto builder = Model::JointChainBuilder();

	Mat4d origin1 = ToTransformMatrix( Vec3d( 0, 0, 0 ) );
	Mat4d origin2 = ToTransformMatrix( Vec3d( 0, 0, 1 ) );

	AddBaseLink( builder );
	AddPrismaticJointLink( builder, 1, origin1, Vec3d::UnitX(), origin2, 0, 1 );
	AddJointTipLink( builder, 2, Model::JointType::REVOLUTE, origin2, Vec3d::UnitX(), home );

	return builder.Build();
}

// ------------------------------------------------------------

Mat4d createPrismaticBaseHome()
{
	return ToTransformMatrix( Vec3d( 0, 1, 1 ) );
}

// ------------------------------------------------------------

Model::KinematicTopology createPrismaticBaseTopology( const Mat4d& home )
{
	Model::KinematicTopology topology;

	Mat4d wrist_center = ToTransformMatrix( Vec3d( 0, 0, 1 ) );

	Model::PrismaticBaseJointGroup base_group( wrist_center );
	Model::WristJointGroup wrist_group( 1, 1, home * Inverse( wrist_center ), wrist_center  );

	topology.Add( base_group );
	topology.Add( wrist_group );

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

	// Add Prismatic joint max length
	total_length += chain.GetActiveJoint( 0 )->GetLimits().Max();

	return Model::Skeleton( articulations, bones, total_length, chain.GetJointCount() );
}

// ------------------------------------------------------------

Model::KinematicModelConstPtr createPrismaticBaseRobot()
{
	auto home = createPrismaticBaseHome();
	auto chain = createPrismaticBaseJointChain( home );
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

Model::JointChainConstPtr createPlanar2RJointChain( const Mat4d& home )
{
	auto builder = Model::JointChainBuilder();

	Mat4d origin1 = ToTransformMatrix( Vec3d(0, 0, 0));
	Mat4d origin2 = ToTransformMatrix( Vec3d(0, 0, 0.5 ));

	AddBaseLink( builder );
	AddRevoluteJointLink( builder, 1, origin1, Vec3d::UnitX(), origin2, -M_PI / 2, M_PI / 2 );
	AddJointTipLink( builder, 2, Model::JointType::REVOLUTE, origin2, Vec3d::UnitX(), home, -M_PI / 2, M_PI / 2 );

	return builder.Build();
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
	Mat4d wrist_center = ToTransformMatrix( Vec3d( 0, 0, 0.5 ) );

	Model::PlanarNRJointGroup planar_group( 0, 1, wrist_center );
	Model::WristJointGroup wrist_group( 1, 1, home * Inverse( wrist_center ), wrist_center  );

	topology.Add( planar_group );
	topology.Add( wrist_group );

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
	auto home = createPlanar2RHome();
	auto chain = createPlanar2RJointChain( home );
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

Model::JointChainConstPtr createPlanar3RJointChain( const Mat4d& home )
{
	auto builder = Model::JointChainBuilder();

	Mat4d origin1 = ToTransformMatrix( Vec3d( 0, 0, 0 ) );
	Mat4d origin2 = ToTransformMatrix( Vec3d( 0, 0, 0.5 ) );
	Mat4d origin3 = ToTransformMatrix( Vec3d( 0, 0, 1.0 ) );

	AddBaseLink( builder );
	AddRevoluteJointLink( builder, 1, origin1, Vec3d::UnitX(), origin2, -M_PI_2, M_PI / 2 );
	AddRevoluteJointLink( builder, 2, origin2, Vec3d::UnitX(), origin3, -M_PI_2, M_PI / 2 );
	AddJointTipLink( builder, 3, Model::JointType::REVOLUTE, origin3, Vec3d::UnitX(), home, -M_PI_2, M_PI / 2 );

	return builder.Build();
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

	Mat4d wrist_center = ToTransformMatrix( Vec3d( 0, 0, 1.0 ) );

	Model::PlanarNRJointGroup planar_group( 0, 2, wrist_center );
	Model::WristJointGroup wrist_group( 2, 1, home * Inverse( wrist_center ), wrist_center  );

	topology.Add( planar_group );
	topology.Add( wrist_group );

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
	auto home = createPlanar3RHome();
	auto chain = createPlanar3RJointChain( home );
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

Model::JointChainConstPtr createWrist1RJointChain( const Mat4d& home )
{
	auto builder = Model::JointChainBuilder();

	Mat4d origin1 = ToTransformMatrix( Vec3d( 0, 0, 1 ) );

	AddBaseLink( builder );
	AddJointTipLink( builder, 1, Model::JointType::REVOLUTE, origin1, Vec3d::UnitX(), home, 0, M_PI );

	return builder.Build();
}

// ------------------------------------------------------------

Mat4d createWrist1RHome()
{
	return ToTransformMatrix( Vec3d( 0, 1, 1 ) );
}

// ------------------------------------------------------------

Model::KinematicTopology createWrist1RTopology( const Mat4d& home )
{
	Model::KinematicTopology topology;

	Mat4d wrist_center = ToTransformMatrix( Vec3d( 0, 0, 1 ) );
	Model::WristJointGroup wrist_group( 0, 1, home * Inverse( wrist_center ), wrist_center  );

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

Model::KinematicModelConstPtr createWrist1RRobot()
{
	auto home = createWrist1RHome();
	auto chain = createWrist1RJointChain( home );
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

Model::JointChainConstPtr createWrist2RJointChain( const Mat4d& home )
{
	auto builder = Model::JointChainBuilder();

	Mat4d origin1 = ToTransformMatrix( Vec3d( 0, 0, 1 ) );

	AddBaseLink( builder );
	AddRevoluteJointLink( builder, 1, origin1, Vec3d::UnitX(), origin1, 0, M_PI );
	AddJointTipLink( builder, 2, Model::JointType::REVOLUTE, origin1, Vec3d::UnitY(), home, -M_PI / 3, M_PI );

	return builder.Build();
}

// ------------------------------------------------------------

Mat4d createWrist2RHome()
{
	return ToTransformMatrix( Vec3d( 0.0, 0, 2 ) );
}

// ------------------------------------------------------------

Model::KinematicTopology createWrist2RTopology( const Mat4d& home )
{
	Model::KinematicTopology topology;

	Mat4d wrist_center = ToTransformMatrix( Vec3d( 0, 0, 1 ) );

	Model::WristJointGroup wrist_group( 0, 2, home * Inverse( wrist_center ), wrist_center  );

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

Model::KinematicModelConstPtr createWrist2RRobot()
{
	auto home = createWrist2RHome();
	auto chain = createWrist2RJointChain( home );
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

Model::JointChainConstPtr createSphericalWristJointChain( const Mat4d& home )
{
	auto builder = Model::JointChainBuilder();

	Mat4d origin1 = ToTransformMatrix( Vec3d( 0, 0, 1 ) );

	AddBaseLink( builder );
	AddRevoluteJointLink( builder, 1, origin1, Vec3d::UnitX(), origin1, -M_PI / 2, M_PI / 2 );
	AddRevoluteJointLink( builder, 2, origin1, Vec3d::UnitY(), origin1, -M_PI / 2, M_PI / 2 );
	AddJointTipLink( builder, 3, Model::JointType::REVOLUTE, origin1, Vec3d::UnitZ(), home, -M_PI / 2, M_PI / 2 );

	return builder.Build();
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

	Mat4d wrist_center = ToTransformMatrix( Vec3d( 0, 0, 1 ) );
	Model::WristJointGroup wrist_group( 0, 3, home * Inverse( wrist_center ), wrist_center  );

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
	auto home = createSphericalWristHome();
	auto chain = createSphericalWristJointChain( home );
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

Model::JointChainConstPtr createRevolute_Planar2R_Wrist2R_5DOFsJointChain( const Mat4d& home )
{
	auto builder = Model::JointChainBuilder();

	Mat4d origin1 = ToTransformMatrix( Vec3d( 0, 0, 0 ) );
	Mat4d origin2 = ToTransformMatrix( Vec3d( 0, 0, 0.5 ) );
	Mat4d origin3 = ToTransformMatrix( Vec3d( 0, 0, 1.0 ) );
	Mat4d origin4 = ToTransformMatrix( Vec3d( 0.5, 0, 1.0 ) );
	Mat4d origin5 = ToTransformMatrix( Vec3d( 0.6, 0, 1.0 ) );

	AddBaseLink( builder );
	AddRevoluteJointLink( builder, 1, origin1, Vec3d::UnitZ(), origin2 );
	AddRevoluteJointLink( builder, 2, origin2, Vec3d::UnitY(), origin3 );
	AddRevoluteJointLink( builder, 3, origin3, Vec3d::UnitY(), origin4 );
	AddRevoluteJointLink( builder, 4, origin4, Vec3d::UnitX(), origin5 );
	AddJointTipLink( builder, 5, Model::JointType::REVOLUTE, origin5, Vec3d::UnitZ(), home );

	return builder.Build();
}

// ------------------------------------------------------------

Mat4d createRevolute_Planar2R_Wrist2R_5DOFsHome()
{
	return ToTransformMatrix( Vec3d( 0.7, 0, 0.9 ) );
}

// ------------------------------------------------------------

Model::KinematicTopology createRevolute_Planar2R_Wrist2R_5DOFsTopology( const Mat4d& home )
{
	Model::KinematicTopology topology;

	Mat4d wrist_center = ToTransformMatrix( Vec3d( 0.6, 0, 1.0 ) );

	Model::RevoluteBaseJointGroup base_group( wrist_center );
	Model::PlanarNRJointGroup planar_group( 1, 2, wrist_center );
	Model::WristJointGroup wrist_group( 3, 2, home * Inverse( wrist_center ), wrist_center  );

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
			Vec3d{ 0.6, 0, 1.0 }
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
	auto home = createRevolute_Planar2R_Wrist2R_5DOFsHome();
	auto chain = createRevolute_Planar2R_Wrist2R_5DOFsJointChain( home );
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

Model::JointChainConstPtr createRevolute_Planar2R_SphericalWrist_6DOFsJointChain( const Mat4d& home )
{
	auto builder = Model::JointChainBuilder();

	Mat4d origin1 = ToTransformMatrix( Vec3d( 0, 0, 0 ) );
	Mat4d origin2 = ToTransformMatrix( Vec3d( 0.5, 0, 0.1 ) );
	Mat4d origin3 = ToTransformMatrix( Vec3d( 1.0, 0, 0.1 ) );
	Mat4d origin4 = ToTransformMatrix( Vec3d( 1.5, 0, 0.1 ) );

	AddBaseLink( builder );
	AddRevoluteJointLink( builder, 1, origin1, Vec3d::UnitZ(), origin2 );
	AddRevoluteJointLink( builder, 2, origin2, Vec3d::UnitY(), origin3 );
	AddRevoluteJointLink( builder, 3, origin3, Vec3d::UnitY(), origin4 );
	AddRevoluteJointLink( builder, 4, origin4, Vec3d::UnitX(), origin4 );
	AddRevoluteJointLink( builder, 5, origin4, Vec3d::UnitY(), origin4 );
	AddJointTipLink( builder, 6, Model::JointType::REVOLUTE, origin4, Vec3d::UnitZ(), home );

	return builder.Build();
}

// ------------------------------------------------------------

Mat4d createRevolute_Planar2R_SphericalWrist_6DOFsHome()
{
	return ToTransformMatrix( Vec3d( 1.5, 0, 0.1 ) );
}

// ------------------------------------------------------------

Model::KinematicTopology createRevolute_Planar2R_SphericalWrist_6DOFsTopology( const Mat4d& home )
{
	Model::KinematicTopology topology;

	Mat4d wrist_center = ToTransformMatrix( Vec3d( 1.5, 0, 0.1 ) );

	Model::RevoluteBaseJointGroup base_group( wrist_center );
	Model::PlanarNRJointGroup planar_group( 1, 2, wrist_center );
	Model::WristJointGroup wrist_group( 3, 3, home * Inverse( wrist_center ), wrist_center  );

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

	bones.emplace_back(
		std::make_shared< const Model::Bone >(
			articulations[2]->Center(),
			articulations[3]->Center() - articulations[2]->Center() )
		);

	double total_length = articulations[0]->Center().norm();
	for ( int i = 0; i < bones.size(); i++ )
		total_length += bones[i]->Length();

	return Model::Skeleton( articulations, bones, total_length, chain.GetJointCount() );
}

// ------------------------------------------------------------

Model::KinematicModelConstPtr createRevolute_Planar2R_SphericalWrist_6DOFsRobot()
{
	auto home = createRevolute_Planar2R_SphericalWrist_6DOFsHome();
	auto chain = createRevolute_Planar2R_SphericalWrist_6DOFsJointChain( home );
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

Model::JointChainConstPtr createURLike_6DOFsJointChain( const Mat4d& home )
{
	auto builder = Model::JointChainBuilder();

	Mat4d origin1 = ToTransformMatrix( Vec3d( 0, 0, 0 ) );
	Mat4d origin2 = ToTransformMatrix( Vec3d( 0, 0.2, 0.2  ) );
	Mat4d origin3 = ToTransformMatrix( Vec3d( 0, 0.2, 0.6 ) );
	Mat4d origin4 = ToTransformMatrix( Vec3d( 0.4, 0, 0.6 ) );
	Mat4d origin5 = ToTransformMatrix( Vec3d( 0.4, 0.2, 0.6 ) );
	Mat4d origin6 = ToTransformMatrix( Vec3d( 0.5, 0.2, 0.6 ) );

	AddBaseLink( builder );
	AddRevoluteJointLink( builder, 1, origin1, Vec3d::UnitZ(), origin2 );
	AddRevoluteJointLink( builder, 2, origin2, Vec3d::UnitY(), origin3, -4 * M_PI / 5, 4 * M_PI / 5 );
	AddRevoluteJointLink( builder, 3, origin3, Vec3d::UnitY(), origin4, -4 * M_PI / 5, 4 * M_PI / 5 );
	AddRevoluteJointLink( builder, 4, origin4, Vec3d::UnitY(), origin5, -4 * M_PI / 5, 4 * M_PI / 5 );
	AddRevoluteJointLink( builder, 5, origin5, Vec3d::UnitX(), origin6, -M_PI / 2, M_PI / 2 );
	AddJointTipLink( builder, 6, Model::JointType::REVOLUTE, origin6, Vec3d::UnitZ(), home );

	return builder.Build();
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
	Model::PlanarNRJointGroup planar_group( 1, 3, wrist_center );
	Model::WristJointGroup wrist_group( 4, 2, home * Inverse( wrist_center ), wrist_center  );

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
	auto home = createURLike_6DOFsHome();
	auto chain = createURLike_6DOFsJointChain( home );
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
// LeRobot
// ------------------------------------------------------------

Model::JointChainConstPtr createLeRobot_JointChain( const Mat4d& home )
{
	auto builder = Model::JointChainBuilder();

	Mat4d link_origin1 = ToTransformMatrix( Vec3d( 0, 0, 0 ) );
	Mat4d link_origin2 = ToTransformMatrix( Vec3d( -9.07886e-05, 0.0590972, 0.031089 ) );
	Mat4d link_origin3 = ToTransformMatrix( Vec3d( -1.72052e-05, 0.0701802, 0.00310545 ) );
	Mat4d link_origin4 = ToTransformMatrix( Vec3d( -0.00339604, 0.00137796, 0.0768007 ) );
	Mat4d link_origin5 = ToTransformMatrix( Vec3d( -0.00852653, -0.0352279, -2.34622e-05 ) );

	Vec2d limits1( -2.0, 2.0 );
	Vec3d joint_axis1 = Vec3d::UnitY();
	Iso3d joint_origin1 = Iso3d::Identity();
	joint_origin1.translate( Vec3d( 0, -0.0452, 0.0165 ) );
	joint_origin1.rotate( AngleAxis( M_PI / 2, Vec3d::UnitX() ) );
	
	Vec2d limits2( 0, 3.5 );
	Vec3d joint_axis2 = Vec3d::UnitX();
	Iso3d joint_origin2 = Iso3d::Identity();
	joint_origin2.translate( Vec3d( 0, 0.1025, 0.0306 ) );
	joint_origin2.rotate( AngleAxis( -1.8, Vec3d::UnitX() ) );
	
	Vec2d limits3( -3.14158, 0 );
	Vec3d joint_axis3 = Vec3d::UnitX();
	Iso3d joint_origin3 = Iso3d::Identity();
	joint_origin3.translate( Vec3d( 0, 0.11257, 0.028 ) );
	joint_origin3.rotate( AngleAxis( M_PI / 2, Vec3d::UnitX() ) );
	
	Vec2d limits4( -2.5, 1.2 );
	Vec3d joint_axis4 = Vec3d::UnitX();
	Iso3d joint_origin4 = Iso3d::Identity();
	joint_origin4.translate( Vec3d( 0, 0.0052, 0.1349 ) );
	joint_origin4.rotate( AngleAxis( -1, Vec3d::UnitX() ) );
	
	Vec2d limits5( -3.14158, 3.14158 );
	Vec3d joint_axis5 = Vec3d::UnitY();
	Iso3d joint_origin5 = Iso3d::Identity();
	joint_origin5.translate( Vec3d( 0, -0.0601, 0 ) );
	joint_origin5.rotate( AngleAxis( M_PI / 2, Vec3d::UnitY() ) );

	AddBaseLink( builder );
	AddRevoluteJointLink( builder, 1, joint_origin1.matrix(), joint_axis1, link_origin2.matrix(), limits1[0], limits1[1] );
	AddRevoluteJointLink( builder, 2, joint_origin2.matrix(), joint_axis2, link_origin3.matrix(), limits2[0], limits2[1] );
	AddRevoluteJointLink( builder, 3, joint_origin3.matrix(), joint_axis3, link_origin4.matrix(), limits3[0], limits3[1] );
	AddRevoluteJointLink( builder, 4, joint_origin4.matrix(), joint_axis4, link_origin5.matrix(), limits4[0], limits4[1] );
	AddJointTipLink( builder, 5, Model::JointType::REVOLUTE, joint_origin5.matrix(), joint_axis5, home, limits5[0], limits5[1] );

	return builder.Build();
}

// ------------------------------------------------------------

Mat4d createLeRobot_Home()
{
	Mat4d home;
	home << 
	6.32679e-06,          0,            1,            0,
	 0.334976,     0.942227, -2.11933e-06,     -0.14663,
	-0.942227,     0.334976,  5.96127e-06,     0.136274,
	        0,            0,            0,            1;
	return home;
}

// ------------------------------------------------------------

Model::KinematicTopology createLeRobot_Topology( const Mat4d& home )
{
	Model::KinematicTopology topology;

	Mat4d wrist_center = ToTransformMatrix( Vec3d( 0, -0.0900018, 0.156406 ) );

	Model::RevoluteBaseJointGroup base_group( wrist_center );
	Model::PlanarNRJointGroup planar_group( 1, 2, wrist_center );
	Model::WristJointGroup wrist_group( 3, 2, home * Inverse( wrist_center ), wrist_center  );

	topology.Add( base_group );
	topology.Add( planar_group );
	topology.Add( wrist_group );

	return topology;
}

// ------------------------------------------------------------

Model::Skeleton createLeRobot_Skeleton(
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

	articulations.emplace_back(
		std::make_shared< const Model::Articulation >(
			Model::ArticulationType::Universal,
			std::vector< Model::JointConstPtr > { chain.GetActiveJoint(  3 ), chain.GetActiveJoint(  4 ) },
			chain.GetActiveJoint( 3 )->Origin()
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
			Translation( home ) - articulations[3]->Center() ) );

	double total_length = articulations[0]->Center().norm();
	for ( int i = 0; i < bones.size(); i++ )
		total_length += bones[i]->Length();

	return Model::Skeleton( articulations, bones, total_length, chain.GetJointCount() );
}

// ------------------------------------------------------------

Model::KinematicModelConstPtr createLeRobot_Robot()
{
	auto home = createLeRobot_Home();
	auto chain = createLeRobot_JointChain( home );
	auto topology = createLeRobot_Topology( home );
	auto skeleton = std::make_shared< const Model::Skeleton >( createLeRobot_Skeleton( *chain, home ) );
	auto reachable_space = createSkeletonReachableSpacePtr( *skeleton );
	return std::make_shared< const Model::KinematicModel >(
		std::move( chain ),
		home,
		topology,
		skeleton,
		std::move( reachable_space ) );
}

// ------------------------------------------------------------

Model::KinematicModelConstPtr Data::GetLeRobot()
{
	if ( !LeRobot_robot_ )
	{
		LeRobot_robot_ = createLeRobot_Robot();
	}
	return LeRobot_robot_;
}

// ------------------------------------------------------------
// LeRobot With Gripper
// ------------------------------------------------------------

Model::JointChainConstPtr createLeRobotWithGripper_JointChain( const Mat4d& home )
{
	// auto gripper_joint = Model::Joint(
	// 	"gripper",
	// 	Model::Twist(),
	// 	Model::Link( "gripper_link", ToTransformMatrix( Vec3d( 0, -0.240852,  0.102777 ) ), 0 ),
	// 	Model::Limits()
	// );

	return createLeRobot_JointChain( home );
}

// ------------------------------------------------------------

Mat4d createLeRobotWithGripper_Home()
{
	Mat4d home;
	home << 
	6.32679e-06,            0,            1,            0,
	   0.334976,     0.942227, -2.11933e-06,    -0.240852,
	  -0.942227,     0.334976,  5.96127e-06,     0.102777,
	          0,            0,            0,            1;
	return home;
}

// ------------------------------------------------------------

Model::KinematicTopology createLeRobotWithGripper_Topology( const Mat4d& home )
{
	Model::KinematicTopology topology;

	// Mat4d wrist_center = ToTransformMatrix( Vec3d( 0.5, 0.2, 0.6 ) );

	// Model::RevoluteBaseJointGroup base_group( wrist_center );
	// Model::PlanarNRJointGroup planar_group( 1, 3, wrist_center );
	// Model::WristJointGroup wrist_group( 4, 2, home * Inverse( wrist_center ), wrist_center  );

	// topology.Add( base_group );
	// topology.Add( planar_group );
	// topology.Add( wrist_group );

	return topology;
}

// ------------------------------------------------------------

Model::Skeleton createLeRobotWithGripper_Skeleton(
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

	articulations.emplace_back(
		std::make_shared< const Model::Articulation >(
			Model::ArticulationType::Universal,
			std::vector< Model::JointConstPtr > { chain.GetActiveJoint(  3 ), chain.GetActiveJoint(  4 ) },
			chain.GetActiveJoint( 3 )->Origin()
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
			articulations[4]->Center() - articulations[3]->Center() )
		);

	bones.emplace_back(
		std::make_shared< const Model::Bone >(
			articulations[4]->Center(),
			Translation( home ) - articulations[4]->Center() ) );

	double total_length = articulations[0]->Center().norm();
	for ( int i = 0; i < bones.size(); i++ )
		total_length += bones[i]->Length();

	return Model::Skeleton( articulations, bones, total_length, chain.GetJointCount() );
}

// ------------------------------------------------------------

Model::KinematicModelConstPtr createLeRobotWithGripper_Robot()
{
	auto home = createLeRobotWithGripper_Home();
	auto chain = createLeRobotWithGripper_JointChain( home );
	auto topology = createLeRobotWithGripper_Topology( home );
	auto skeleton = std::make_shared< const Model::Skeleton >( createLeRobotWithGripper_Skeleton( *chain, home ) );
	auto reachable_space = createSkeletonReachableSpacePtr( *skeleton );
	return std::make_shared< const Model::KinematicModel >(
		std::move( chain ),
		home,
		topology,
		skeleton,
		std::move( reachable_space ) );
}

// ------------------------------------------------------------

Model::KinematicModelConstPtr Data::GetLeRobotWithGripper()
{
	throw std::invalid_argument( "No model implemented for LeRobotWithGripper" );
	if ( !LeRobotWithGripper_robot_ )
	{
		LeRobotWithGripper_robot_ = createLeRobotWithGripper_Robot();
	}
	return LeRobotWithGripper_robot_;
}

// ------------------------------------------------------------

std::map< std::string, Model::KinematicModelConstPtr > Data::GetAllRobots()
{
	return
	    {
			{ "ZYZ", GetZYZRevoluteRobot() },
			{ "RevoluteBase", GetRevoluteBaseRobot() },
			{ "PrismaticBase", GetPrismaticBaseRobot() },
			{ "Planar2R", GetPlanar2RRobot() },
			{ "Planar3R", GetPlanar3RRobot() },
			{ "Wrist1R", GetWrist1RRobot() },
			{ "Wrist2R", GetWrist2RRobot() },
			{ "Wrist3R", GetSphericalWristRobot() },
			{ "5-axis arm", GetRevolute_Planar2R_Wrist2R_5DOFsRobot() },
			{ "6-axis arm", GetRevolute_Planar2R_SphericalWrist_6DOFsRobot() },
			{ "Universal Robot", GetURLikeRobot() },
			//{ "LeRobot", GetLeRobot() },
			//{ "LeRobotWithGripper", GetLeRobotWithGripper() },
		};
}

// ------------------------------------------------------------

}