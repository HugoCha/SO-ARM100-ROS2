#include "Model/Articulation.hpp"

#include "Global.hpp"
#include "KinematicTestBase.hpp"
#include "Model/ArticulationType.hpp"
#include "Model/Joint.hpp"
#include "RobotModelTestData.hpp"

#include "Model/JointChain.hpp"
#include "Model/Limits.hpp"
#include "Model/Link.hpp"
#include "Model/Twist.hpp"
#include "Utils/Converter.hpp"

#include <gtest/gtest.h>

namespace SOArm100::Kinematics::Test
{

// ------------------------------------------------------------
// ------------------------------------------------------------

class ArticulationTest : public KinematicTestBase
{
protected:
void SetUp() override
{
}

void TearDown() override
{
}
};

// ------------------------------------------------------------
// ------------------------------------------------------------

TEST_F( ArticulationTest, ExtractFromJoints_Planar2RWithTipDifferentThanLastJointAxis_ReturnExpected )
{
	auto robot = Data::GetPlanar2RRobot();
	auto joints = robot->GetChain()->GetJoints();
	auto articulations = Model::Articulation::ExtractFromJoints(
		joints,
		robot->GetHomeConfiguration() );

	EXPECT_EQ( joints.size(), articulations.size() )
	    << "Expected Size : " << joints.size() << " Result Size : " << articulations.size() << std::endl;
	EXPECT_EQ( 1, articulations[0]->JointCount() );
	EXPECT_EQ( 1, articulations[1]->JointCount() );
	EXPECT_EQ( 1,  articulations[0]->Joints().size() );
	EXPECT_EQ( 1,  articulations[1]->Joints().size() );

	EXPECT_EQ( joints[0],  articulations[0]->Joints()[0] );
	EXPECT_EQ( joints[1],  articulations[1]->Joints()[0] );

	EXPECT_EQ( Model::ArticulationType::Revolute, articulations[0]->GetType() );
	EXPECT_EQ( Model::ArticulationType::Revolute, articulations[1]->GetType() );

	EXPECT_EQ( joints[0]->Axis(), articulations[0]->Axis() );
	EXPECT_EQ( joints[1]->Axis(), articulations[1]->Axis() );

	EXPECT_EQ( joints[0]->Origin(), articulations[0]->Center() );
	EXPECT_EQ( joints[1]->Origin(), articulations[1]->Center() );
}

// ------------------------------------------------------------

TEST_F( ArticulationTest, ExtractFromJoints_Planar3RTipOnLastJointAxis_ReturnExpected )
{
	auto robot = Data::GetPlanar3RRobot();
	auto joints = robot->GetChain()->GetJoints();
	auto tip_home = robot->GetHomeConfiguration();
	auto articulations = Model::Articulation::ExtractFromJoints(
		joints,
		tip_home );

	EXPECT_EQ( joints.size(), articulations.size() )
	    << "Expected Size : " << joints.size() << " Result Size : " << articulations.size() << std::endl;
	EXPECT_EQ( 1, articulations[0]->JointCount() );
	EXPECT_EQ( 1, articulations[1]->JointCount() );
	EXPECT_EQ( 1, articulations[2]->JointCount() );
	EXPECT_EQ( 1,  articulations[0]->Joints().size() );
	EXPECT_EQ( 1,  articulations[1]->Joints().size() );
	EXPECT_EQ( 1,  articulations[2]->Joints().size() );

	EXPECT_EQ( joints[0],  articulations[0]->Joints()[0] );
	EXPECT_EQ( joints[1],  articulations[1]->Joints()[0] );
	EXPECT_EQ( joints[2],  articulations[2]->Joints()[0] );

	EXPECT_EQ( Model::ArticulationType::Revolute, articulations[0]->GetType() );
	EXPECT_EQ( Model::ArticulationType::Revolute, articulations[1]->GetType() );
	EXPECT_EQ( Model::ArticulationType::Revolute, articulations[2]->GetType() );

	EXPECT_EQ( joints[0]->Axis(), articulations[0]->Axis() );
	EXPECT_EQ( joints[1]->Axis(), articulations[1]->Axis() );
	EXPECT_EQ( joints[2]->Axis(), articulations[2]->Axis() );

	EXPECT_EQ( joints[0]->Origin(), articulations[0]->Center() );
	EXPECT_EQ( joints[1]->Origin(), articulations[1]->Center() );
	// Articulation center is shifted to Tip position
	// because tip is on Last Joint axis
	EXPECT_EQ( Translation( tip_home ), articulations[2]->Center() );
}

// ------------------------------------------------------------

TEST_F( ArticulationTest, ExtractFromJoints_PrismaticBaseReturnExpected )
{
	auto robot = Data::GetPrismaticBaseRobot();
	auto joints = robot->GetChain()->GetJoints();
	auto articulations = Model::Articulation::ExtractFromJoints(
		joints,
		robot->GetHomeConfiguration() );

	EXPECT_EQ( joints.size(), articulations.size() )
	    << "Expected Size : " << joints.size() << " Result Size : " << articulations.size() << std::endl;
	EXPECT_EQ( 1, articulations[0]->JointCount() );
	EXPECT_EQ( 1, articulations[1]->JointCount() );
	EXPECT_EQ( 1,  articulations[0]->Joints().size() );
	EXPECT_EQ( 1,  articulations[1]->Joints().size() );

	EXPECT_EQ( joints[0],  articulations[0]->Joints()[0] );
	EXPECT_EQ( joints[1],  articulations[1]->Joints()[0] );

	EXPECT_EQ( Model::ArticulationType::Prismatic, articulations[0]->GetType() );
	EXPECT_EQ( Model::ArticulationType::Revolute, articulations[1]->GetType() );

	EXPECT_EQ( joints[0]->Axis(), articulations[0]->Axis() );
	EXPECT_EQ( joints[1]->Axis(), articulations[1]->Axis() );

	EXPECT_EQ( joints[0]->Origin(), articulations[0]->Center() );
	EXPECT_EQ( joints[1]->Origin(), articulations[1]->Center() );
}

// ------------------------------------------------------------

TEST_F( ArticulationTest, ExtractFromJoints_RevoluteBaseReturnExpected )
{
	auto robot = Data::GetRevoluteBaseRobot();
	auto joints = robot->GetChain()->GetJoints();
	auto tip_home = robot->GetHomeConfiguration();
	auto articulations = Model::Articulation::ExtractFromJoints(
		joints,
		tip_home );

	EXPECT_EQ( 2, articulations.size() )
	    << "Expected Size : " << 2 << " Result Size : " << articulations.size() << std::endl;
	EXPECT_EQ( 2, articulations[0]->JointCount() );
	EXPECT_EQ( 1, articulations[1]->JointCount() );
	EXPECT_EQ( 2,  articulations[0]->Joints().size() );
	EXPECT_EQ( 1,  articulations[1]->Joints().size() );

	EXPECT_EQ( Model::ArticulationType::Universal, articulations[0]->GetType() );
	EXPECT_EQ( Model::ArticulationType::Revolute, articulations[1]->GetType() );

	EXPECT_EQ( joints[1]->Axis(), articulations[0]->Axis() );
	EXPECT_EQ( joints[2]->Axis(), articulations[1]->Axis() );

	EXPECT_EQ( joints[1]->Origin(), articulations[0]->Center() );
	// Articulation center is shifted to Tip position
	// because tip is on Last Joint axis
	EXPECT_EQ( Translation( tip_home ), articulations[1]->Center() );
}

// ------------------------------------------------------------

TEST_F( ArticulationTest, ExtractFromJoints_SphericalWristReturnExpected )
{
	auto robot = Data::GetSphericalWristRobot();
	auto joints = robot->GetChain()->GetJoints();
	auto tip_home = robot->GetHomeConfiguration();
	auto articulations = Model::Articulation::ExtractFromJoints(
		joints,
		robot->GetHomeConfiguration() );

	EXPECT_EQ( 1, articulations.size() )
	    << "Expected Size : " << 2 << " Result Size : " << articulations.size() << std::endl;
	EXPECT_EQ( 3, articulations[0]->JointCount() );
	EXPECT_EQ( 3,  articulations[0]->Joints().size() );

	EXPECT_EQ( Model::ArticulationType::Spherical, articulations[0]->GetType() );

	EXPECT_EQ( joints[2]->Axis(), articulations[0]->Axis() );
	EXPECT_TRUE( joints[0]->Origin().isApprox( articulations[0]->Center() ) );
}

// ------------------------------------------------------------

TEST_F( ArticulationTest, ExtractFromJoints_5DofsReturnExpected )
{
	auto robot = Data::GetRevolute_Planar2R_Wrist2R_5DOFsRobot();
	auto joints = robot->GetChain()->GetJoints();
	auto articulations = Model::Articulation::ExtractFromJoints(
		joints,
		robot->GetHomeConfiguration() );

	EXPECT_EQ( 3, articulations.size() )
	    << "Expected Size : " << joints.size() << " Result Size : " << articulations.size() << std::endl;
	EXPECT_EQ( 2, articulations[0]->JointCount() );
	EXPECT_EQ( 1, articulations[1]->JointCount() );
	EXPECT_EQ( 2, articulations[2]->JointCount() );

	EXPECT_EQ( Model::ArticulationType::Universal, articulations[0]->GetType() );
	EXPECT_EQ( Model::ArticulationType::Revolute, articulations[1]->GetType() );
	EXPECT_EQ( Model::ArticulationType::Universal, articulations[2]->GetType() );

	EXPECT_EQ( joints[1]->Axis(), articulations[0]->Axis() );
	EXPECT_EQ( joints[2]->Axis(), articulations[1]->Axis() );
	EXPECT_EQ( joints[4]->Axis(), articulations[2]->Axis() );

	EXPECT_EQ( joints[1]->Origin(), articulations[0]->Center() );
	EXPECT_EQ( joints[2]->Origin(), articulations[1]->Center() );
	EXPECT_EQ( joints[3]->Origin(), articulations[2]->Center() );
}

// ------------------------------------------------------------

TEST_F( ArticulationTest, ExtractFromJoints_6DofWithSphericalWristReturnExpected )
{
	auto robot = Data::GetRevolute_Planar2R_SphericalWrist_6DOFsRobot();
	auto joints = robot->GetChain()->GetJoints();
	auto articulations = Model::Articulation::ExtractFromJoints(
		joints,
		robot->GetHomeConfiguration() );

	EXPECT_EQ( 3, articulations.size() )
	    << "Expected Size : " << joints.size() << " Result Size : " << articulations.size() << std::endl;
	EXPECT_EQ( 2, articulations[0]->JointCount() );
	EXPECT_EQ( 1, articulations[1]->JointCount() );
	EXPECT_EQ( 3, articulations[2]->JointCount() );

	EXPECT_EQ( Model::ArticulationType::Universal, articulations[0]->GetType() );
	EXPECT_EQ( Model::ArticulationType::Revolute, articulations[1]->GetType() );
	EXPECT_EQ( Model::ArticulationType::Spherical, articulations[2]->GetType() );

	EXPECT_EQ( joints[1]->Axis(), articulations[0]->Axis() );
	EXPECT_EQ( joints[2]->Axis(), articulations[1]->Axis() );
	EXPECT_EQ( joints[5]->Axis(), articulations[2]->Axis() );

	EXPECT_EQ( joints[1]->Origin(), articulations[0]->Center() );
	EXPECT_EQ( joints[2]->Origin(), articulations[1]->Center() );
	EXPECT_EQ( joints[3]->Origin(), articulations[2]->Center() );
}

// ------------------------------------------------------------

TEST_F( ArticulationTest, ExtractFromJoints_6DofWithNonSphericalWristReturnExpected )
{
	auto robot = Data::GetURLikeRobot();
	auto joints = robot->GetChain()->GetJoints();
	auto articulations = Model::Articulation::ExtractFromJoints(
		joints,
		robot->GetHomeConfiguration() );

	EXPECT_EQ( 4, articulations.size() )
	    << "Expected Size : " << joints.size() << " Result Size : " << articulations.size() << std::endl;
	EXPECT_EQ( 2, articulations[0]->JointCount() );
	EXPECT_EQ( 1, articulations[1]->JointCount() );
	EXPECT_EQ( 1, articulations[2]->JointCount() );
	EXPECT_EQ( 2, articulations[3]->JointCount() );

	EXPECT_EQ( Model::ArticulationType::Universal, articulations[0]->GetType() );
	EXPECT_EQ( Model::ArticulationType::Revolute, articulations[1]->GetType() );
	EXPECT_EQ( Model::ArticulationType::Revolute, articulations[2]->GetType() );
	EXPECT_EQ( Model::ArticulationType::Universal, articulations[3]->GetType() );

	EXPECT_EQ( joints[1]->Axis(), articulations[0]->Axis() );
	EXPECT_EQ( joints[2]->Axis(), articulations[1]->Axis() );
	EXPECT_EQ( joints[3]->Axis(), articulations[2]->Axis() );
	EXPECT_EQ( joints[5]->Axis(), articulations[3]->Axis() );

	EXPECT_EQ(
		Vec3d(
			joints[0]->Origin().x(),
			joints[0]->Origin().y(),
			joints[1]->Origin().z() ),
		articulations[0]->Center() )
	    << "Result   = " << articulations[0]->Center().transpose() << std::endl
	    << "Expected = " << Vec3d(
		joints[0]->Origin().x(),
		joints[0]->Origin().y(),
		joints[1]->Origin().z() ).transpose() << std::endl;
	EXPECT_EQ( joints[2]->Origin(), articulations[1]->Center() );
	EXPECT_EQ( joints[3]->Origin(), articulations[2]->Center() );
	EXPECT_EQ(
		Vec3d(
			joints[5]->Origin().x(),
			joints[4]->Origin().y(),
			joints[4]->Origin().z() ),
		articulations[3]->Center() )
	    << "Result   = " << articulations[3]->Center().transpose() << std::endl
	    << "Expected = " << Vec3d(
		joints[5]->Origin().x(),
		joints[4]->Origin().y(),
		joints[4]->Origin().z() ).transpose() << std::endl;
}

// ------------------------------------------------------------

TEST_F( ArticulationTest, ExtractFromJoints_ZYZRobotReturnExpected )
{
	auto robot = Data::GetZYZRevoluteRobot();
	auto joints = robot->GetChain()->GetJoints();
	auto articulations = Model::Articulation::ExtractFromJoints(
		joints,
		robot->GetHomeConfiguration() );

	EXPECT_EQ( 3, articulations.size() )
	    << "Expected Size : " << joints.size() << " Result Size : " << articulations.size() << std::endl;
	EXPECT_EQ( 1, articulations[0]->JointCount() );
	EXPECT_EQ( 1, articulations[1]->JointCount() );
	EXPECT_EQ( 1, articulations[2]->JointCount() );

	EXPECT_EQ( Model::ArticulationType::Revolute, articulations[0]->GetType() );
	EXPECT_EQ( Model::ArticulationType::Revolute, articulations[1]->GetType() );
	EXPECT_EQ( Model::ArticulationType::Revolute, articulations[2]->GetType() );

	EXPECT_EQ( joints[0]->Axis(), articulations[0]->Axis() );
	EXPECT_EQ( joints[1]->Axis(), articulations[1]->Axis() );
	EXPECT_EQ( joints[2]->Axis(), articulations[2]->Axis() );

	EXPECT_EQ( joints[0]->Origin(), articulations[0]->Center() );
	EXPECT_EQ( joints[1]->Origin(), articulations[1]->Center() );
	EXPECT_EQ( joints[2]->Origin(), articulations[2]->Center() );
}

// ------------------------------------------------------------

TEST_F( ArticulationTest, ExtractFromJoints_3ZRobot_HomeOnJointAxis_ReturnExpected )
{
	auto chain = Model::JointChain( 3 );

	chain.Add(
		Model::Twist( Vec3d::UnitZ(), Vec3d::Zero() ),
		Model::Link( Mat4d::Identity(), 0.5 ),
		Model::Limits()
		);

	chain.Add(
		Model::Twist( Vec3d::UnitZ(), Vec3d( 0, 0, 0.5 ) ),
		Model::Link( ToTransformMatrix( Vec3d( 0, 0, 0.5 ) ), 0.5 ),
		Model::Limits()
		);

	chain.Add(
		Model::Twist( Vec3d::UnitZ(), Vec3d( 0, 0, 1 ) ),
		Model::Link( ToTransformMatrix( Vec3d( 0, 0, 1 ) ), 0 ),
		Model::Limits()
		);

	Mat4d home = ToTransformMatrix( Vec3d( 0, 0, 1.5 ) );

	auto joints = chain.GetJoints();
	auto articulations = Model::Articulation::ExtractFromJoints(
		joints,
		home );

	EXPECT_EQ( 3, articulations.size() )
	    << "Expected Size : " << joints.size() << " Result Size : " << articulations.size() << std::endl;
	EXPECT_EQ( 1, articulations[0]->JointCount() );
	EXPECT_EQ( 1, articulations[1]->JointCount() );
	EXPECT_EQ( 1, articulations[2]->JointCount() );

	EXPECT_EQ( Model::ArticulationType::Revolute, articulations[0]->GetType() );
	EXPECT_EQ( Model::ArticulationType::Revolute, articulations[1]->GetType() );
	EXPECT_EQ( Model::ArticulationType::Revolute, articulations[2]->GetType() );

	EXPECT_EQ( joints[0]->Axis(), articulations[0]->Axis() );
	EXPECT_EQ( joints[1]->Axis(), articulations[1]->Axis() );
	EXPECT_EQ( joints[2]->Axis(), articulations[2]->Axis() );

	EXPECT_EQ( joints[0]->Origin(), articulations[0]->Center() );
	EXPECT_EQ( joints[1]->Origin(), articulations[1]->Center() );
	EXPECT_EQ( joints[2]->Origin(), articulations[2]->Center() );
}

// ------------------------------------------------------------

TEST_F( ArticulationTest, ExtractFromJoints_7DofsRobot_ReturnExpected )
{
	auto chain = std::make_unique< Model::JointChain >( 7 );

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

	// Wrist 2 revolute joints
	origin = next_origin;
	next_origin = Vec3d( 0.4, 0.2, 0.6 );
	axis = Vec3d::UnitY();
	chain->Add(
		Model::Twist( axis, origin ),
		Model::Link( ToTransformMatrix( origin ), ToTransformMatrix( next_origin ) ),
		Model::Limits( -M_PI, M_PI )
		);

	origin = next_origin;
	next_origin = Vec3d( 0.5, 0.2, 0.6 );
	axis = Vec3d::UnitX();
	chain->Add(
		Model::Twist( axis, origin ),
		Model::Link( ToTransformMatrix( origin ), ToTransformMatrix( next_origin ) ),
		Model::Limits( -M_PI, M_PI )
		);

	origin = next_origin;
	next_origin = Vec3d( 0.5, 0.2, 0.5 );
	axis = Vec3d::UnitZ();
	chain->Add(
		Model::Twist( axis, origin ),
		Model::Link( ToTransformMatrix( origin ), ToTransformMatrix( next_origin ) ),
		Model::Limits( -M_PI, M_PI )
		);


	origin = next_origin;
	axis = Vec3d::UnitX();
	chain->Add(
		Model::Twist( axis, origin ),
		Model::Link( ToTransformMatrix( origin ), ToTransformMatrix( next_origin ) ),
		Model::Limits( -M_PI, M_PI )
		);

	Mat4d home = ToTransformMatrix( Vec3d( 0.5, 0.2, 0.4 ) );

}

// ------------------------------------------------------------

TEST_F( ArticulationTest, ExtractFromJoints_3ZRobot_HomeDifferentLastJointAxis_ReturnExpected )
{
	auto chain = Model::JointChain( 3 );

	chain.Add(
		Model::Twist( Vec3d::UnitZ(), Vec3d::Zero() ),
		Model::Link( Mat4d::Identity(), 0.5 ),
		Model::Limits()
		);

	chain.Add(
		Model::Twist( Vec3d::UnitZ(), Vec3d( 0, 0, 0.5 ) ),
		Model::Link( ToTransformMatrix( Vec3d( 0, 0, 0.5 ) ), 0.5 ),
		Model::Limits()
		);

	chain.Add(
		Model::Twist( Vec3d::UnitZ(), Vec3d( 0, 0, 1 ) ),
		Model::Link( ToTransformMatrix( Vec3d( 0, 0, 1 ) ), 0 ),
		Model::Limits()
		);

	Mat4d home = ToTransformMatrix( Vec3d( 0.1, 0, 1 ) );

}

// ------------------------------------------------------------

}