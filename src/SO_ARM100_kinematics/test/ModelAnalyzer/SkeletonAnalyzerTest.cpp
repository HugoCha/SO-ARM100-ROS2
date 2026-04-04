#include "ModelAnalyzer/SkeletonAnalyzer.hpp"

#include "Global.hpp"
#include "KinematicTestBase.hpp"
#include "RobotModelTestData.hpp"

#include "Utils/Converter.hpp"
#include "Utils/KinematicsUtils.hpp"

#include <gtest/gtest.h>

namespace SOArm100::Kinematics::Test
{

// ------------------------------------------------------------
// ------------------------------------------------------------

class SkeletonTest : public KinematicTestBase
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
// Analyze Articulation
// ------------------------------------------------------------

void MakeAzimuthChain( double link_length, int count, Model::JointChain& chain, Mat4d& tip )
{
	Vec3d axis;
	Vec3d origin = Vec3d::Zero();
	for ( int i = 0; i < count; i++ )
	{
		Vec3d axis = i % 2 == 0 ? Vec3d::UnitZ() : Vec3d::UnitY();
		chain.Add(
			Model::Twist( axis, origin ),
			Model::Link( ToTransformMatrix( origin ), link_length ),
			Model::Limits( -M_PI, M_PI ) );

		if ( i % 2 == 0 )
		{
			origin.z() += link_length;
		}
		else
		{
			origin.y() += link_length;
		}
	}

	tip = ToTransformMatrix( origin );
}

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

TEST_F( ArticulationTest, AnalyzeArticulations_Planar2RWithTipDifferentThanLastJointAxis_ReturnExpected )
{
	auto robot = Data::GetPlanar2RRobot();
	auto joints = robot->GetChain()->GetJoints();
	auto articulations = Model::SkeletonAnalyzer::AnalyzeArticulations(
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

TEST_F( ArticulationTest, AnalyzeArticulations_Planar3RTipOnLastJointAxis_ReturnExpected )
{
	auto robot = Data::GetPlanar3RRobot();
	auto joints = robot->GetChain()->GetJoints();
	auto tip_home = robot->GetHomeConfiguration();
	auto articulations = Model::SkeletonAnalyzer::AnalyzeArticulations(
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

TEST_F( ArticulationTest, AnalyzeArticulations_PrismaticBaseReturnExpected )
{
	auto robot = Data::GetPrismaticBaseRobot();
	auto joints = robot->GetChain()->GetJoints();
	auto articulations = Model::SkeletonAnalyzer::AnalyzeArticulations(
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

TEST_F( ArticulationTest, AnalyzeArticulations_RevoluteBaseReturnExpected )
{
	auto robot = Data::GetRevoluteBaseRobot();
	auto joints = robot->GetChain()->GetJoints();
	auto tip_home = robot->GetHomeConfiguration();
	auto articulations = Model::SkeletonAnalyzer::AnalyzeArticulations(
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

TEST_F( ArticulationTest, AnalyzeArticulations_SphericalWristReturnExpected )
{
	auto robot = Data::GetSphericalWristRobot();
	auto joints = robot->GetChain()->GetJoints();
	auto tip_home = robot->GetHomeConfiguration();
	auto articulations = Model::SkeletonAnalyzer::AnalyzeArticulations(
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

TEST_F( ArticulationTest, AnalyzeArticulations_5DofsReturnExpected )
{
	auto robot = Data::GetRevolute_Planar2R_Wrist2R_5DOFsRobot();
	auto joints = robot->GetChain()->GetJoints();
	auto articulations = Model::SkeletonAnalyzer::AnalyzeArticulations(
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

TEST_F( ArticulationTest, AnalyzeArticulations_6DofWithSphericalWristReturnExpected )
{
	auto robot = Data::GetRevolute_Planar2R_SphericalWrist_6DOFsRobot();
	auto joints = robot->GetChain()->GetJoints();
	auto articulations = Model::SkeletonAnalyzer::AnalyzeArticulations(
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

TEST_F( ArticulationTest, AnalyzeArticulations_6DofWithNonSphericalWristReturnExpected )
{
	auto robot = Data::GetURLikeRobot();
	auto joints = robot->GetChain()->GetJoints();
	auto articulations = Model::SkeletonAnalyzer::AnalyzeArticulations(
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

TEST_F( ArticulationTest, AnalyzeArticulations_ZYZRobotReturnExpected )
{
	auto robot = Data::GetZYZRevoluteRobot();
	auto joints = robot->GetChain()->GetJoints();
	auto articulations = Model::SkeletonAnalyzer::AnalyzeArticulations(
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

TEST_F( ArticulationTest, AnalyzeArticulations_3ZRobot_HomeOnJointAxis_ReturnExpected )
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
	auto articulations = Model::SkeletonAnalyzer::AnalyzeArticulations(
		joints,
		home );

	EXPECT_EQ( 1, articulations.size() )
	    << "Expected Size : " << 1 << " Result Size : " << articulations.size() << std::endl;
	EXPECT_EQ( 1, articulations[0]->JointCount() );
	EXPECT_EQ( joints[0],  articulations[0]->Joints()[0] );

	EXPECT_EQ( Model::ArticulationType::Revolute, articulations[0]->GetType() );

	EXPECT_EQ( joints[0]->Axis(), articulations[0]->Axis() );

	EXPECT_EQ( Translation( home ), articulations[0]->Center() );
}

// ------------------------------------------------------------

TEST_F( ArticulationTest, AnalyzeArticulations_2Joints_AzimuthRobot_ReturnExpected )
{
	int count = 2;
	double link_length = 0.5;
	Model::JointChain chain( count );
	Mat4d tip;

	MakeAzimuthChain( link_length, count, chain, tip );

	auto joints = chain.GetJoints();
	auto articulations = Model::SkeletonAnalyzer::AnalyzeArticulations(
		joints,
		tip );

	EXPECT_EQ( 1, articulations.size() );

	EXPECT_EQ( 2, articulations[0]->JointCount() );

	EXPECT_EQ( Model::ArticulationType::Universal, articulations[0]->GetType() );

	EXPECT_EQ( joints[1]->Axis(), articulations[0]->Axis() );

	EXPECT_EQ( joints[1]->Origin(), articulations[0]->Center() );
}

// ------------------------------------------------------------

TEST_F( ArticulationTest, AnalyzeArticulations_3Joints_AzimuthRobot_ReturnExpected )
{
	int count = 3;
	double link_length = 0.5;
	Model::JointChain chain( count );
	Mat4d tip;

	MakeAzimuthChain( link_length, count, chain, tip );

	auto joints = chain.GetJoints();
	auto articulations = Model::SkeletonAnalyzer::AnalyzeArticulations(
		joints,
		tip );

	EXPECT_EQ( 2, articulations.size() );

	EXPECT_EQ( 1, articulations[0]->JointCount() );
	EXPECT_EQ( 2, articulations[1]->JointCount() );

	EXPECT_EQ( Model::ArticulationType::Revolute, articulations[0]->GetType() );
	EXPECT_EQ( Model::ArticulationType::Universal, articulations[1]->GetType() );

	EXPECT_EQ( joints[0]->Axis(), articulations[0]->Axis() );
	EXPECT_EQ( joints[2]->Axis(), articulations[1]->Axis() );

	EXPECT_EQ( joints[0]->Origin(), articulations[0]->Center() );
	EXPECT_EQ( joints[2]->Origin(), articulations[1]->Center() );
}

// ------------------------------------------------------------

TEST_F( ArticulationTest, AnalyzeArticulations_4Joints_AzimuthRobot_ReturnExpected )
{
	int count = 4;
	double link_length = 0.5;
	Model::JointChain chain( count );
	Mat4d tip;

	MakeAzimuthChain( link_length, count, chain, tip );

	auto joints = chain.GetJoints();
	auto articulations = Model::SkeletonAnalyzer::AnalyzeArticulations(
		joints,
		tip );

	EXPECT_EQ( 2, articulations.size() );

	EXPECT_EQ( 2, articulations[0]->JointCount() );
	EXPECT_EQ( 2, articulations[1]->JointCount() );

	EXPECT_EQ( Model::ArticulationType::Universal, articulations[0]->GetType() );
	EXPECT_EQ( Model::ArticulationType::Universal, articulations[1]->GetType() );

	EXPECT_EQ( joints[1]->Axis(), articulations[0]->Axis() );
	EXPECT_EQ( joints[3]->Axis(), articulations[1]->Axis() );

	EXPECT_EQ( joints[1]->Origin(), articulations[0]->Center() );
	EXPECT_EQ( joints[3]->Origin(), articulations[1]->Center() );
}

// ------------------------------------------------------------

TEST_F( ArticulationTest, AnalyzeArticulations_PairCountJoints_AzimuthRobot_ReturnExpected )
{
	int count = 10;
	double link_length = 0.5;
	Model::JointChain chain( count );
	Mat4d tip;

	MakeAzimuthChain( link_length, count, chain, tip );

	auto joints = chain.GetJoints();
	auto articulations = Model::SkeletonAnalyzer::AnalyzeArticulations(
		joints,
		tip );

	EXPECT_EQ( count / 2, articulations.size() );

	for ( int i = 0; i < count / 2; i++ )
	{
		EXPECT_EQ( 2, articulations[i]->JointCount() );
		EXPECT_EQ( Model::ArticulationType::Universal, articulations[i]->GetType() );
		EXPECT_EQ( joints[2 * i + 1]->Axis(), articulations[i]->Axis() );
		EXPECT_EQ( joints[2 * i + 1]->Origin(), articulations[i]->Center() );
	}
}

// ------------------------------------------------------------

TEST_F( ArticulationTest, AnalyzeArticulations_OddCountJoints_AzimuthRobot_ReturnExpected )
{
	int count = 11;
	double link_length = 0.5;
	Model::JointChain chain( count );
	Mat4d tip;

	MakeAzimuthChain( link_length, count, chain, tip );

	auto joints = chain.GetJoints();
	auto articulations = Model::SkeletonAnalyzer::AnalyzeArticulations(
		joints,
		tip );

	int articulation_count = count / 2 + 1;
	EXPECT_EQ( articulation_count, articulations.size() );

	EXPECT_EQ( 1, articulations[0]->JointCount() );
	EXPECT_EQ( Model::ArticulationType::Revolute, articulations[0]->GetType() );
	EXPECT_EQ( joints[0]->Axis(), articulations[0]->Axis() );
	EXPECT_EQ( joints[0]->Origin(), articulations[0]->Center() );

	for ( int i = 1; i < articulation_count; i++ )
	{
		EXPECT_EQ( 2, articulations[i]->JointCount() );
		EXPECT_EQ( Model::ArticulationType::Universal, articulations[i]->GetType() );
		EXPECT_EQ( joints[2 * i]->Axis(), articulations[i]->Axis() );
		EXPECT_EQ( joints[2 * i]->Origin(), articulations[i]->Center() );
	}
}

// ------------------------------------------------------------
// Analyze Skeleton
// ------------------------------------------------------------

TEST_F( SkeletonTest, Analyze_Planar2RWithTipDifferentThanLastJointAxis_ReturnExpected )
{
	auto robot = Data::GetPlanar2RRobot();
	auto skeleton = Model::SkeletonAnalyzer::Analyze( robot->GetChain()->GetJoints(), robot->GetHomeConfiguration() );

    int articulation_count = skeleton->ArticulationCount();
    EXPECT_EQ( 2, articulation_count );
    EXPECT_EQ( 2, skeleton->BonesCount() );

    for ( int i = 0; i < skeleton->BonesCount() - 1; i++ )
    {
        Vec3d expected_bone = skeleton->Center( i + 1) - skeleton->Center( i );
        EXPECT_EQ( expected_bone, skeleton->Bone( i )->Direction() );
        EXPECT_EQ( expected_bone.norm(), skeleton->Bone( i )->Length() );
    }

    Vec3d expected_last_bone = 
        Translation( robot->GetHomeConfiguration() ) - skeleton->Center( articulation_count - 1 );
    EXPECT_EQ( expected_last_bone, skeleton->Bone( skeleton->BonesCount() - 1 )->Direction() );
    EXPECT_EQ( expected_last_bone.norm(), skeleton->Bone( skeleton->BonesCount() - 1 )->Length() );
}

// ------------------------------------------------------------

TEST_F( SkeletonTest, Analyze_Planar3RTipOnLastJointAxis_ReturnExpected )
{
	auto robot = Data::GetPlanar3RRobot();
    auto skeleton = Model::SkeletonAnalyzer::Analyze( robot->GetChain()->GetJoints(), robot->GetHomeConfiguration() );

    int articulation_count = skeleton->ArticulationCount();
    EXPECT_EQ( 3, articulation_count );
    EXPECT_EQ( 2, skeleton->BonesCount() );

    for ( int i = 0; i < skeleton->BonesCount(); i++ )
    {
        Vec3d expected_bone = skeleton->Center( i + 1) - skeleton->Center( i );
        EXPECT_EQ( expected_bone, skeleton->Bone( i )->Direction() );
        EXPECT_EQ( expected_bone.norm(), skeleton->Bone( i )->Length() );
    }
}

// ------------------------------------------------------------

TEST_F( SkeletonTest, Analyze_PrismaticBaseReturnExpected )
{
	auto robot = Data::GetPrismaticBaseRobot();
    auto skeleton = Model::SkeletonAnalyzer::Analyze( robot->GetChain()->GetJoints(), robot->GetHomeConfiguration() );

    int articulation_count = skeleton->ArticulationCount();
    EXPECT_EQ( 2, articulation_count );
    EXPECT_EQ( 2, skeleton->BonesCount() );

    for ( int i = 0; i < skeleton->BonesCount() - 1; i++ )
    {
        Vec3d expected_bone = skeleton->Center( i + 1) - skeleton->Center( i );
        EXPECT_EQ( expected_bone, skeleton->Bone( i )->Direction() );
        EXPECT_EQ( expected_bone.norm(), skeleton->Bone( i )->Length() );
    }

    Vec3d expected_last_bone = 
        Translation( robot->GetHomeConfiguration() ) - skeleton->Center( articulation_count - 1 );
    EXPECT_EQ( expected_last_bone, skeleton->Bone( skeleton->BonesCount() - 1 )->Direction() );
    EXPECT_EQ( expected_last_bone.norm(), skeleton->Bone( skeleton->BonesCount() - 1 )->Length() );
}

// ------------------------------------------------------------

TEST_F( SkeletonTest, Analyze_RevoluteBaseReturnExpected )
{
	auto robot = Data::GetRevoluteBaseRobot();
    auto skeleton = Model::SkeletonAnalyzer::Analyze( robot->GetChain()->GetJoints(), robot->GetHomeConfiguration() );

    int articulation_count = skeleton->ArticulationCount();
    EXPECT_EQ( 2, articulation_count );
    EXPECT_EQ( 1, skeleton->BonesCount() );

    for ( int i = 0; i < skeleton->BonesCount(); i++ )
    {
        Vec3d expected_bone = skeleton->Center( i + 1) - skeleton->Center( i );
        EXPECT_EQ( expected_bone, skeleton->Bone( i )->Direction() );
        EXPECT_EQ( expected_bone.norm(), skeleton->Bone( i )->Length() );
    }
}

// ------------------------------------------------------------

TEST_F( SkeletonTest, Analyze_SphericalWristReturnExpected )
{
	auto robot = Data::GetSphericalWristRobot();
    auto skeleton = Model::SkeletonAnalyzer::Analyze( robot->GetChain()->GetJoints(), robot->GetHomeConfiguration() );

    int articulation_count = skeleton->ArticulationCount();
    EXPECT_EQ( 1, articulation_count );
    EXPECT_EQ( 1, skeleton->BonesCount() );

    Vec3d expected_last_bone = 
        Translation( robot->GetHomeConfiguration() ) - skeleton->Center( articulation_count - 1 );
    EXPECT_EQ( expected_last_bone, skeleton->Bone( skeleton->BonesCount() - 1 )->Direction() );
    EXPECT_EQ( expected_last_bone.norm(), skeleton->Bone( skeleton->BonesCount() - 1 )->Length() );
}

// ------------------------------------------------------------

TEST_F( SkeletonTest, Analyze_5DofsReturnExpected )
{
	auto robot = Data::GetRevolute_Planar2R_Wrist2R_5DOFsRobot();
    auto skeleton = Model::SkeletonAnalyzer::Analyze( robot->GetChain()->GetJoints(), robot->GetHomeConfiguration() );

    int articulation_count = skeleton->ArticulationCount();
    EXPECT_EQ( 3, articulation_count );
    EXPECT_EQ( 3, skeleton->BonesCount() );

    for ( int i = 0; i < skeleton->BonesCount() - 1; i++ )
    {
        Vec3d expected_bone = skeleton->Center( i + 1) - skeleton->Center( i );
        EXPECT_EQ( expected_bone, skeleton->Bone( i )->Direction() );
        EXPECT_EQ( expected_bone.norm(), skeleton->Bone( i )->Length() );
    }

    Vec3d expected_last_bone = 
        Translation( robot->GetHomeConfiguration() ) - skeleton->Center( articulation_count - 1 );
    EXPECT_EQ( expected_last_bone, skeleton->Bone( skeleton->BonesCount() - 1 )->Direction() );
    EXPECT_EQ( expected_last_bone.norm(), skeleton->Bone( skeleton->BonesCount() - 1 )->Length() );
}

// ------------------------------------------------------------

TEST_F( SkeletonTest, Analyze_6DofWithSphericalWristReturnExpected )
{
	auto robot = Data::GetRevolute_Planar2R_SphericalWrist_6DOFsRobot();
    auto skeleton = Model::SkeletonAnalyzer::Analyze( robot->GetChain()->GetJoints(), robot->GetHomeConfiguration() );

    int articulation_count = skeleton->ArticulationCount();
    EXPECT_EQ( 3, articulation_count );
    EXPECT_EQ( 2, skeleton->BonesCount() );

    for ( int i = 0; i < skeleton->BonesCount(); i++ )
    {
        Vec3d expected_bone = skeleton->Center( i + 1) - skeleton->Center( i );
        EXPECT_EQ( expected_bone, skeleton->Bone( i )->Direction() );
        EXPECT_EQ( expected_bone.norm(), skeleton->Bone( i )->Length() );
    }
}

// ------------------------------------------------------------

TEST_F( SkeletonTest, Analyze_6DofWithNonSphericalWristReturnExpected )
{
	auto robot = Data::GetURLikeRobot();
    auto skeleton = Model::SkeletonAnalyzer::Analyze( robot->GetChain()->GetJoints(), robot->GetHomeConfiguration() );

    int articulation_count = skeleton->ArticulationCount();
    EXPECT_EQ( 4, articulation_count );
    EXPECT_EQ( 4, skeleton->BonesCount() );

    for ( int i = 0; i < skeleton->BonesCount() - 1; i++ )
    {
        Vec3d expected_bone = skeleton->Center( i + 1) - skeleton->Center( i );
        EXPECT_EQ( expected_bone, skeleton->Bone( i )->Direction() );
        EXPECT_EQ( expected_bone.norm(), skeleton->Bone( i )->Length() );
    }

    Vec3d expected_last_bone = 
        Translation( robot->GetHomeConfiguration() ) - skeleton->Center( articulation_count - 1 );
    EXPECT_EQ( expected_last_bone, skeleton->Bone( skeleton->BonesCount() - 1 )->Direction() );
    EXPECT_EQ( expected_last_bone.norm(), skeleton->Bone( skeleton->BonesCount() - 1 )->Length() );
}

// ------------------------------------------------------------

TEST_F( SkeletonTest, Analyze_ZYZRobotReturnExpected )
{
	auto robot = Data::GetZYZRevoluteRobot();
    auto skeleton = Model::SkeletonAnalyzer::Analyze( robot->GetChain()->GetJoints(), robot->GetHomeConfiguration() );

    int articulation_count = skeleton->ArticulationCount();
    EXPECT_EQ( 3, articulation_count );
    EXPECT_EQ( 2, skeleton->BonesCount() );

    for ( int i = 0; i < skeleton->BonesCount(); i++ )
    {
        Vec3d expected_bone = skeleton->Center( i + 1) - skeleton->Center( i );
        EXPECT_EQ( expected_bone, skeleton->Bone( i )->Direction() );
        EXPECT_EQ( expected_bone.norm(), skeleton->Bone( i )->Length() );
    }
}

// ------------------------------------------------------------

}