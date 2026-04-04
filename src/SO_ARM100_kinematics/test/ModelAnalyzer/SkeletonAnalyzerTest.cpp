#include "ModelAnalyzer/SkeletonAnalyzer.hpp"

#include "Global.hpp"
#include "KinematicTestBase.hpp"
#include "RobotModelTestData.hpp"

#include "Utils/Converter.hpp"
#include "Utils/KinematicsUtils.hpp"
#include "Utils/StringConverter.hpp"

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
	auto skeleton = robot->GetSkeleton();
	auto articulations = Model::SkeletonAnalyzer::AnalyzeArticulations(
		joints,
		robot->GetHomeConfiguration() );

	EXPECT_EQ( skeleton->ArticulationCount(), articulations.size() );

	for ( int i = 0; i < skeleton->ArticulationCount(); i++ )
	{
		EXPECT_EQ( skeleton->Articulation( i )->GetType(), articulations[i]->GetType() );
		EXPECT_EQ( skeleton->Articulation( i )->Axis(), articulations[i]->Axis() );
		EXPECT_EQ( skeleton->Articulation( i )->Center(), articulations[i]->Center() );
		EXPECT_EQ( skeleton->Articulation( i )->JointCount(), articulations[i]->JointCount() );
	}
}

// ------------------------------------------------------------

TEST_F( ArticulationTest, AnalyzeArticulations_Planar3RTipOnLastJointAxis_ReturnExpected )
{
	auto robot = Data::GetPlanar3RRobot();
	auto joints = robot->GetChain()->GetJoints();
	auto tip_home = robot->GetHomeConfiguration();
	auto skeleton = robot->GetSkeleton();
	auto articulations = Model::SkeletonAnalyzer::AnalyzeArticulations(
		joints,
		robot->GetHomeConfiguration() );

	EXPECT_EQ( skeleton->ArticulationCount(), articulations.size() );

	for ( int i = 0; i < skeleton->ArticulationCount(); i++ )
	{
		EXPECT_EQ( skeleton->Articulation( i )->GetType(), articulations[i]->GetType() );
		EXPECT_EQ( skeleton->Articulation( i )->Axis(), articulations[i]->Axis() );
		EXPECT_EQ( skeleton->Articulation( i )->Center(), articulations[i]->Center() );
		EXPECT_EQ( skeleton->Articulation( i )->JointCount(), articulations[i]->JointCount() );
	}
}

// ------------------------------------------------------------

TEST_F( ArticulationTest, AnalyzeArticulations_PrismaticBaseReturnExpected )
{
	auto robot = Data::GetPrismaticBaseRobot();
	auto joints = robot->GetChain()->GetJoints();
	auto skeleton = robot->GetSkeleton();
	auto articulations = Model::SkeletonAnalyzer::AnalyzeArticulations(
		joints,
		robot->GetHomeConfiguration() );

	EXPECT_EQ( skeleton->ArticulationCount(), articulations.size() );

	for ( int i = 0; i < skeleton->ArticulationCount(); i++ )
	{
		EXPECT_EQ( skeleton->Articulation( i )->GetType(), articulations[i]->GetType() );
		EXPECT_EQ( skeleton->Articulation( i )->Axis(), articulations[i]->Axis() );
		EXPECT_EQ( skeleton->Articulation( i )->Center(), articulations[i]->Center() );
		EXPECT_EQ( skeleton->Articulation( i )->JointCount(), articulations[i]->JointCount() );
	}
}

// ------------------------------------------------------------

TEST_F( ArticulationTest, AnalyzeArticulations_RevoluteBaseReturnExpected )
{
	auto robot = Data::GetRevoluteBaseRobot();
	auto joints = robot->GetChain()->GetJoints();
	auto tip_home = robot->GetHomeConfiguration();
	auto skeleton = robot->GetSkeleton();
	auto articulations = Model::SkeletonAnalyzer::AnalyzeArticulations(
		joints,
		robot->GetHomeConfiguration() );

	EXPECT_EQ( skeleton->ArticulationCount(), articulations.size() );

	for ( int i = 0; i < skeleton->ArticulationCount(); i++ )
	{
		EXPECT_EQ( skeleton->Articulation( i )->GetType(), articulations[i]->GetType() );
		EXPECT_EQ( skeleton->Articulation( i )->Axis(), articulations[i]->Axis() );
		EXPECT_EQ( skeleton->Articulation( i )->Center(), articulations[i]->Center() );
		EXPECT_EQ( skeleton->Articulation( i )->JointCount(), articulations[i]->JointCount() );
	}
}

// ------------------------------------------------------------

TEST_F( ArticulationTest, AnalyzeArticulations_SphericalWristReturnExpected )
{
	auto robot = Data::GetSphericalWristRobot();
	auto joints = robot->GetChain()->GetJoints();
	auto tip_home = robot->GetHomeConfiguration();
	auto skeleton = robot->GetSkeleton();
	auto articulations = Model::SkeletonAnalyzer::AnalyzeArticulations(
		joints,
		robot->GetHomeConfiguration() );

	EXPECT_EQ( skeleton->ArticulationCount(), articulations.size() );

	for ( int i = 0; i < skeleton->ArticulationCount(); i++ )
	{
		EXPECT_EQ( skeleton->Articulation( i )->GetType(), articulations[i]->GetType() );
		EXPECT_EQ( skeleton->Articulation( i )->Axis(), articulations[i]->Axis() );
		EXPECT_EQ( skeleton->Articulation( i )->Center(), articulations[i]->Center() );
		EXPECT_EQ( skeleton->Articulation( i )->JointCount(), articulations[i]->JointCount() );
	}
}

// ------------------------------------------------------------

TEST_F( ArticulationTest, AnalyzeArticulations_5DofsReturnExpected )
{
	auto robot = Data::GetRevolute_Planar2R_Wrist2R_5DOFsRobot();
	auto joints = robot->GetChain()->GetJoints();
	auto skeleton = robot->GetSkeleton();
	auto articulations = Model::SkeletonAnalyzer::AnalyzeArticulations(
		joints,
		robot->GetHomeConfiguration() );

	EXPECT_EQ( skeleton->ArticulationCount(), articulations.size() );

	for ( int i = 0; i < skeleton->ArticulationCount(); i++ )
	{
		EXPECT_EQ( skeleton->Articulation( i )->GetType(), articulations[i]->GetType() );
		EXPECT_EQ( skeleton->Articulation( i )->Axis(), articulations[i]->Axis() );
		EXPECT_EQ( skeleton->Articulation( i )->Center(), articulations[i]->Center() );
		EXPECT_EQ( skeleton->Articulation( i )->JointCount(), articulations[i]->JointCount() );
	}
}

// ------------------------------------------------------------

TEST_F( ArticulationTest, AnalyzeArticulations_6DofWithSphericalWristReturnExpected )
{
	auto robot = Data::GetRevolute_Planar2R_SphericalWrist_6DOFsRobot();
	auto joints = robot->GetChain()->GetJoints();
	auto skeleton = robot->GetSkeleton();
	auto articulations = Model::SkeletonAnalyzer::AnalyzeArticulations(
		joints,
		robot->GetHomeConfiguration() );

	EXPECT_EQ( skeleton->ArticulationCount(), articulations.size() );

	for ( int i = 0; i < skeleton->ArticulationCount(); i++ )
	{
		EXPECT_EQ( skeleton->Articulation( i )->GetType(), articulations[i]->GetType() );
		EXPECT_EQ( skeleton->Articulation( i )->Axis(), articulations[i]->Axis() );
		EXPECT_EQ( skeleton->Articulation( i )->Center(), articulations[i]->Center() );
		EXPECT_EQ( skeleton->Articulation( i )->JointCount(), articulations[i]->JointCount() );
	}
}

// ------------------------------------------------------------

TEST_F( ArticulationTest, AnalyzeArticulations_6DofWithNonSphericalWristReturnExpected )
{
	auto robot = Data::GetURLikeRobot();
	auto joints = robot->GetChain()->GetJoints();
	auto skeleton = robot->GetSkeleton();
	auto articulations = Model::SkeletonAnalyzer::AnalyzeArticulations(
		joints,
		robot->GetHomeConfiguration() );

	EXPECT_EQ( skeleton->ArticulationCount(), articulations.size() );

	for ( int i = 0; i < skeleton->ArticulationCount(); i++ )
	{
		EXPECT_EQ( skeleton->Articulation( i )->GetType(), articulations[i]->GetType() );
		EXPECT_EQ( skeleton->Articulation( i )->Axis(), articulations[i]->Axis() );
		EXPECT_EQ( skeleton->Articulation( i )->Center(), articulations[i]->Center() );
		EXPECT_EQ( skeleton->Articulation( i )->JointCount(), articulations[i]->JointCount() );
	}
}

// ------------------------------------------------------------

TEST_F( ArticulationTest, AnalyzeArticulations_ZYZRobotReturnExpected )
{
	auto robot = Data::GetZYZRevoluteRobot();
	auto joints = robot->GetChain()->GetJoints();
	auto skeleton = robot->GetSkeleton();
	auto articulations = Model::SkeletonAnalyzer::AnalyzeArticulations(
		joints,
		robot->GetHomeConfiguration() );

	EXPECT_EQ( skeleton->ArticulationCount(), articulations.size() );

	for ( int i = 0; i < skeleton->ArticulationCount(); i++ )
	{
		EXPECT_EQ( skeleton->Articulation( i )->GetType(), articulations[i]->GetType() );
		EXPECT_EQ( skeleton->Articulation( i )->Axis(), articulations[i]->Axis() );
		EXPECT_EQ( skeleton->Articulation( i )->Center(), articulations[i]->Center() );
		EXPECT_EQ( skeleton->Articulation( i )->JointCount(), articulations[i]->JointCount() );
	}
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
	auto joints = robot->GetChain()->GetJoints();
	auto expected_skeleton = robot->GetSkeleton();
	auto result_skeleton = Model::SkeletonAnalyzer::Analyze(
		joints,
		robot->GetHomeConfiguration() );

	EXPECT_EQ( expected_skeleton->JointCount(), result_skeleton->JointCount() );
	EXPECT_EQ( expected_skeleton->TotalLength(), result_skeleton->TotalLength() );
	EXPECT_EQ( expected_skeleton->ArticulationCount(), result_skeleton->ArticulationCount() );

	for ( int i = 0; i < expected_skeleton->ArticulationCount(); i++ )
	{
		EXPECT_EQ( expected_skeleton->Articulation( i )->GetType(),   result_skeleton->Articulation( i )->GetType() );
		EXPECT_EQ( expected_skeleton->Articulation( i )->Axis(),      result_skeleton->Articulation( i )->Axis() );
		EXPECT_EQ( expected_skeleton->Articulation( i )->Center(),    result_skeleton->Articulation( i )->Center() );
		EXPECT_EQ( expected_skeleton->Articulation( i )->JointCount(),result_skeleton->Articulation( i )->JointCount() );
	}

	EXPECT_EQ( expected_skeleton->BonesCount(), result_skeleton->BonesCount() );

	for ( int i = 0; i < expected_skeleton->BonesCount(); i++ )
	{
		EXPECT_EQ( expected_skeleton->Bone( i )->Origin(),    result_skeleton->Bone( i )->Origin() );
		EXPECT_EQ( expected_skeleton->Bone( i )->Direction(), result_skeleton->Bone( i )->Direction() );
		EXPECT_EQ( expected_skeleton->Bone( i )->Length(),    result_skeleton->Bone( i )->Length() );
	}
}

// ------------------------------------------------------------

TEST_F( SkeletonTest, Analyze_Planar3RTipOnLastJointAxis_ReturnExpected )
{
	auto robot = Data::GetPlanar3RRobot();
	auto joints = robot->GetChain()->GetJoints();
	auto expected_skeleton = robot->GetSkeleton();
	auto result_skeleton = Model::SkeletonAnalyzer::Analyze(
		joints,
		robot->GetHomeConfiguration() );

	EXPECT_EQ( expected_skeleton->JointCount(), result_skeleton->JointCount() );
	EXPECT_EQ( expected_skeleton->TotalLength(), result_skeleton->TotalLength() );
	EXPECT_EQ( expected_skeleton->ArticulationCount(), result_skeleton->ArticulationCount() );

	for ( int i = 0; i < expected_skeleton->ArticulationCount(); i++ )
	{
		EXPECT_EQ( expected_skeleton->Articulation( i )->GetType(),   result_skeleton->Articulation( i )->GetType() );
		EXPECT_EQ( expected_skeleton->Articulation( i )->Axis(),      result_skeleton->Articulation( i )->Axis() );
		EXPECT_EQ( expected_skeleton->Articulation( i )->Center(),    result_skeleton->Articulation( i )->Center() );
		EXPECT_EQ( expected_skeleton->Articulation( i )->JointCount(),result_skeleton->Articulation( i )->JointCount() );
	}

	EXPECT_EQ( expected_skeleton->BonesCount(), result_skeleton->BonesCount() );

	for ( int i = 0; i < expected_skeleton->BonesCount(); i++ )
	{
		EXPECT_EQ( expected_skeleton->Bone( i )->Origin(),    result_skeleton->Bone( i )->Origin() );
		EXPECT_EQ( expected_skeleton->Bone( i )->Direction(), result_skeleton->Bone( i )->Direction() );
		EXPECT_EQ( expected_skeleton->Bone( i )->Length(),    result_skeleton->Bone( i )->Length() );
	}
}

// ------------------------------------------------------------

TEST_F( SkeletonTest, Analyze_PrismaticBaseReturnExpected )
{
	auto robot = Data::GetPrismaticBaseRobot();
	auto joints = robot->GetChain()->GetJoints();
	auto expected_skeleton = robot->GetSkeleton();
	auto result_skeleton = Model::SkeletonAnalyzer::Analyze(
		joints,
		robot->GetHomeConfiguration() );

	EXPECT_EQ( expected_skeleton->JointCount(), result_skeleton->JointCount() );
	EXPECT_EQ( expected_skeleton->TotalLength(), result_skeleton->TotalLength() );
	EXPECT_EQ( expected_skeleton->ArticulationCount(), result_skeleton->ArticulationCount() );

	for ( int i = 0; i < expected_skeleton->ArticulationCount(); i++ )
	{
		EXPECT_EQ( expected_skeleton->Articulation( i )->GetType(),   result_skeleton->Articulation( i )->GetType() );
		EXPECT_EQ( expected_skeleton->Articulation( i )->Axis(),      result_skeleton->Articulation( i )->Axis() );
		EXPECT_EQ( expected_skeleton->Articulation( i )->Center(),    result_skeleton->Articulation( i )->Center() );
		EXPECT_EQ( expected_skeleton->Articulation( i )->JointCount(),result_skeleton->Articulation( i )->JointCount() );
	}

	EXPECT_EQ( expected_skeleton->BonesCount(), result_skeleton->BonesCount() );

	for ( int i = 0; i < expected_skeleton->BonesCount(); i++ )
	{
		EXPECT_EQ( expected_skeleton->Bone( i )->Origin(),    result_skeleton->Bone( i )->Origin() );
		EXPECT_EQ( expected_skeleton->Bone( i )->Direction(), result_skeleton->Bone( i )->Direction() );
		EXPECT_EQ( expected_skeleton->Bone( i )->Length(),    result_skeleton->Bone( i )->Length() );
	}
}

// ------------------------------------------------------------

TEST_F( SkeletonTest, Analyze_RevoluteBaseReturnExpected )
{
	auto robot = Data::GetRevoluteBaseRobot();
	auto joints = robot->GetChain()->GetJoints();
	auto expected_skeleton = robot->GetSkeleton();
	auto result_skeleton = Model::SkeletonAnalyzer::Analyze(
		joints,
		robot->GetHomeConfiguration() );

	EXPECT_EQ( expected_skeleton->JointCount(), result_skeleton->JointCount() );
	EXPECT_EQ( expected_skeleton->TotalLength(), result_skeleton->TotalLength() );
	EXPECT_EQ( expected_skeleton->ArticulationCount(), result_skeleton->ArticulationCount() );

	for ( int i = 0; i < expected_skeleton->ArticulationCount(); i++ )
	{
		EXPECT_EQ( expected_skeleton->Articulation( i )->GetType(),   result_skeleton->Articulation( i )->GetType() );
		EXPECT_EQ( expected_skeleton->Articulation( i )->Axis(),      result_skeleton->Articulation( i )->Axis() );
		EXPECT_EQ( expected_skeleton->Articulation( i )->Center(),    result_skeleton->Articulation( i )->Center() );
		EXPECT_EQ( expected_skeleton->Articulation( i )->JointCount(),result_skeleton->Articulation( i )->JointCount() );
	}

	EXPECT_EQ( expected_skeleton->BonesCount(), result_skeleton->BonesCount() );

	for ( int i = 0; i < expected_skeleton->BonesCount(); i++ )
	{
		EXPECT_EQ( expected_skeleton->Bone( i )->Origin(),    result_skeleton->Bone( i )->Origin() );
		EXPECT_EQ( expected_skeleton->Bone( i )->Direction(), result_skeleton->Bone( i )->Direction() );
		EXPECT_EQ( expected_skeleton->Bone( i )->Length(),    result_skeleton->Bone( i )->Length() );
	}
}

// ------------------------------------------------------------

TEST_F( SkeletonTest, Analyze_SphericalWristReturnExpected )
{
	auto robot = Data::GetSphericalWristRobot();
	auto joints = robot->GetChain()->GetJoints();
	auto expected_skeleton = robot->GetSkeleton();
	auto result_skeleton = Model::SkeletonAnalyzer::Analyze(
		joints,
		robot->GetHomeConfiguration() );

	EXPECT_EQ( expected_skeleton->JointCount(), result_skeleton->JointCount() );
	EXPECT_EQ( expected_skeleton->TotalLength(), result_skeleton->TotalLength() );
	EXPECT_EQ( expected_skeleton->ArticulationCount(), result_skeleton->ArticulationCount() );

	for ( int i = 0; i < expected_skeleton->ArticulationCount(); i++ )
	{
		EXPECT_EQ( expected_skeleton->Articulation( i )->GetType(),   result_skeleton->Articulation( i )->GetType() );
		EXPECT_EQ( expected_skeleton->Articulation( i )->Axis(),      result_skeleton->Articulation( i )->Axis() );
		EXPECT_EQ( expected_skeleton->Articulation( i )->Center(),    result_skeleton->Articulation( i )->Center() );
		EXPECT_EQ( expected_skeleton->Articulation( i )->JointCount(),result_skeleton->Articulation( i )->JointCount() );
	}

	EXPECT_EQ( expected_skeleton->BonesCount(), result_skeleton->BonesCount() );

	for ( int i = 0; i < expected_skeleton->BonesCount(); i++ )
	{
		EXPECT_EQ( expected_skeleton->Bone( i )->Origin(),    result_skeleton->Bone( i )->Origin() );
		EXPECT_EQ( expected_skeleton->Bone( i )->Direction(), result_skeleton->Bone( i )->Direction() );
		EXPECT_EQ( expected_skeleton->Bone( i )->Length(),    result_skeleton->Bone( i )->Length() );
	}
}

// ------------------------------------------------------------

TEST_F( SkeletonTest, Analyze_5DofsReturnExpected )
{
	auto robot = Data::GetRevolute_Planar2R_Wrist2R_5DOFsRobot();
	auto joints = robot->GetChain()->GetJoints();
	auto expected_skeleton = robot->GetSkeleton();
	auto result_skeleton = Model::SkeletonAnalyzer::Analyze(
		joints,
		robot->GetHomeConfiguration() );

	EXPECT_EQ( expected_skeleton->JointCount(), result_skeleton->JointCount() );
	EXPECT_EQ( expected_skeleton->TotalLength(), result_skeleton->TotalLength() );
	EXPECT_EQ( expected_skeleton->ArticulationCount(), result_skeleton->ArticulationCount() );

	for ( int i = 0; i < expected_skeleton->ArticulationCount(); i++ )
	{
		EXPECT_EQ( expected_skeleton->Articulation( i )->GetType(),   result_skeleton->Articulation( i )->GetType() );
		EXPECT_EQ( expected_skeleton->Articulation( i )->Axis(),      result_skeleton->Articulation( i )->Axis() );
		EXPECT_EQ( expected_skeleton->Articulation( i )->Center(),    result_skeleton->Articulation( i )->Center() );
		EXPECT_EQ( expected_skeleton->Articulation( i )->JointCount(),result_skeleton->Articulation( i )->JointCount() );
	}

	EXPECT_EQ( expected_skeleton->BonesCount(), result_skeleton->BonesCount() );

	for ( int i = 0; i < expected_skeleton->BonesCount(); i++ )
	{
		EXPECT_EQ( expected_skeleton->Bone( i )->Origin(),    result_skeleton->Bone( i )->Origin() );
		EXPECT_EQ( expected_skeleton->Bone( i )->Direction(), result_skeleton->Bone( i )->Direction() );
		EXPECT_EQ( expected_skeleton->Bone( i )->Length(),    result_skeleton->Bone( i )->Length() );
	}
}

// ------------------------------------------------------------

TEST_F( SkeletonTest, Analyze_6DofWithSphericalWristReturnExpected )
{
	auto robot = Data::GetRevolute_Planar2R_SphericalWrist_6DOFsRobot();
	auto joints = robot->GetChain()->GetJoints();
	auto expected_skeleton = robot->GetSkeleton();
	auto result_skeleton = Model::SkeletonAnalyzer::Analyze(
		joints,
		robot->GetHomeConfiguration() );

	EXPECT_EQ( expected_skeleton->JointCount(), result_skeleton->JointCount() );
	EXPECT_EQ( expected_skeleton->TotalLength(), result_skeleton->TotalLength() );
	EXPECT_EQ( expected_skeleton->ArticulationCount(), result_skeleton->ArticulationCount() );

	for ( int i = 0; i < expected_skeleton->ArticulationCount(); i++ )
	{
		EXPECT_EQ( expected_skeleton->Articulation( i )->GetType(),   result_skeleton->Articulation( i )->GetType() );
		EXPECT_EQ( expected_skeleton->Articulation( i )->Axis(),      result_skeleton->Articulation( i )->Axis() );
		EXPECT_EQ( expected_skeleton->Articulation( i )->Center(),    result_skeleton->Articulation( i )->Center() );
		EXPECT_EQ( expected_skeleton->Articulation( i )->JointCount(),result_skeleton->Articulation( i )->JointCount() );
	}

	EXPECT_EQ( expected_skeleton->BonesCount(), result_skeleton->BonesCount() );

	for ( int i = 0; i < expected_skeleton->BonesCount(); i++ )
	{
		EXPECT_EQ( expected_skeleton->Bone( i )->Origin(),    result_skeleton->Bone( i )->Origin() );
		EXPECT_EQ( expected_skeleton->Bone( i )->Direction(), result_skeleton->Bone( i )->Direction() );
		EXPECT_EQ( expected_skeleton->Bone( i )->Length(),    result_skeleton->Bone( i )->Length() );
	}
}

// ------------------------------------------------------------

TEST_F( SkeletonTest, Analyze_6DofWithNonSphericalWristReturnExpected )
{
	auto robot = Data::GetURLikeRobot();
	auto joints = robot->GetChain()->GetJoints();
	auto expected_skeleton = robot->GetSkeleton();
	auto result_skeleton = Model::SkeletonAnalyzer::Analyze(
		joints,
		robot->GetHomeConfiguration() );

	EXPECT_EQ( expected_skeleton->JointCount(), result_skeleton->JointCount() );
	EXPECT_EQ( expected_skeleton->TotalLength(), result_skeleton->TotalLength() );
	EXPECT_EQ( expected_skeleton->ArticulationCount(), result_skeleton->ArticulationCount() );

	for ( int i = 0; i < expected_skeleton->ArticulationCount(); i++ )
	{
		EXPECT_EQ( expected_skeleton->Articulation( i )->GetType(),   result_skeleton->Articulation( i )->GetType() );
		EXPECT_EQ( expected_skeleton->Articulation( i )->Axis(),      result_skeleton->Articulation( i )->Axis() );
		EXPECT_EQ( expected_skeleton->Articulation( i )->Center(),    result_skeleton->Articulation( i )->Center() );
		EXPECT_EQ( expected_skeleton->Articulation( i )->JointCount(),result_skeleton->Articulation( i )->JointCount() );
	}

	EXPECT_EQ( expected_skeleton->BonesCount(), result_skeleton->BonesCount() );

	for ( int i = 0; i < expected_skeleton->BonesCount(); i++ )
	{
		EXPECT_EQ( expected_skeleton->Bone( i )->Origin(),    result_skeleton->Bone( i )->Origin() );
		EXPECT_EQ( expected_skeleton->Bone( i )->Direction(), result_skeleton->Bone( i )->Direction() );
		EXPECT_EQ( expected_skeleton->Bone( i )->Length(),    result_skeleton->Bone( i )->Length() );
	}
}

// ------------------------------------------------------------

TEST_F( SkeletonTest, Analyze_ZYZRobotReturnExpected )
{
	auto robot = Data::GetZYZRevoluteRobot();
	auto joints = robot->GetChain()->GetJoints();
	auto expected_skeleton = robot->GetSkeleton();
	auto result_skeleton = Model::SkeletonAnalyzer::Analyze(
		joints,
		robot->GetHomeConfiguration() );

	EXPECT_EQ( expected_skeleton->JointCount(), result_skeleton->JointCount() );
	EXPECT_EQ( expected_skeleton->TotalLength(), result_skeleton->TotalLength() );
	EXPECT_EQ( expected_skeleton->ArticulationCount(), result_skeleton->ArticulationCount() );

	for ( int i = 0; i < expected_skeleton->ArticulationCount(); i++ )
	{
		EXPECT_EQ( expected_skeleton->Articulation( i )->GetType(),   result_skeleton->Articulation( i )->GetType() );
		EXPECT_EQ( expected_skeleton->Articulation( i )->Axis(),      result_skeleton->Articulation( i )->Axis() );
		EXPECT_EQ( expected_skeleton->Articulation( i )->Center(),    result_skeleton->Articulation( i )->Center() );
		EXPECT_EQ( expected_skeleton->Articulation( i )->JointCount(),result_skeleton->Articulation( i )->JointCount() );
	}

	EXPECT_EQ( expected_skeleton->BonesCount(), result_skeleton->BonesCount() );

	for ( int i = 0; i < expected_skeleton->BonesCount(); i++ )
	{
		EXPECT_EQ( expected_skeleton->Bone( i )->Origin(),    result_skeleton->Bone( i )->Origin() );
		EXPECT_EQ( expected_skeleton->Bone( i )->Direction(), result_skeleton->Bone( i )->Direction() );
		EXPECT_EQ( expected_skeleton->Bone( i )->Length(),    result_skeleton->Bone( i )->Length() );
	}
}

// ------------------------------------------------------------

}