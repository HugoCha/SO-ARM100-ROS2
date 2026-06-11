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
	Mat4d home = ToTransformMatrix( Vec3d( 1.5, 0, 1.0 ) );
	auto chain = CreateSimpleJointChain(
		{ 
			{ Vec3d(0,0,0), Vec3d::UnitZ() },
			{ Vec3d(0,0,0.5), Vec3d::UnitZ() },
			{ Vec3d(0,0,1.0), Vec3d::UnitZ() },
		},
		home );

	auto joints = chain->GetJoints();
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
		EXPECT_EQ( expected_skeleton->Articulation( i )->JointCount(), result_skeleton->Articulation( i )->JointCount() );
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
		EXPECT_EQ( expected_skeleton->Articulation( i )->JointCount(), result_skeleton->Articulation( i )->JointCount() );
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
		EXPECT_EQ( expected_skeleton->Articulation( i )->JointCount(), result_skeleton->Articulation( i )->JointCount() );
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
		EXPECT_EQ( expected_skeleton->Articulation( i )->JointCount(), result_skeleton->Articulation( i )->JointCount() );
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
		EXPECT_EQ( expected_skeleton->Articulation( i )->JointCount(), result_skeleton->Articulation( i )->JointCount() );
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
		EXPECT_EQ( expected_skeleton->Articulation( i )->JointCount(), result_skeleton->Articulation( i )->JointCount() );
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
		EXPECT_EQ( expected_skeleton->Articulation( i )->JointCount(), result_skeleton->Articulation( i )->JointCount() );
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
		EXPECT_EQ( expected_skeleton->Articulation( i )->JointCount(), result_skeleton->Articulation( i )->JointCount() );
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
		EXPECT_EQ( expected_skeleton->Articulation( i )->JointCount(), result_skeleton->Articulation( i )->JointCount() );
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