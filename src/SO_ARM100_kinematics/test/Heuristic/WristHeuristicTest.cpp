#include "Heuristic/WristHeuristic.hpp"

#include "Global.hpp"
#include "Heuristic/IKHeuristicState.hpp"
#include "RobotModelTestData.hpp"
#include "Solver/IKRunContext.hpp"
#include "KinematicTestBase.hpp"

#include "Heuristic/IKPresolution.hpp"
#include "Model/JointChain.hpp"
#include "Model/JointGroup.hpp"
#include "Model/KinematicModel.hpp"
#include "Utils/Converter.hpp"
#include "Utils/KinematicsUtils.hpp"

#include <Eigen/src/Geometry/AngleAxis.h>
#include <gtest/gtest.h>
#include <ostream>

namespace SOArm100::Kinematics::Test
{

// ------------------------------------------------------------
// ------------------------------------------------------------

class WristHeuristicTest : public KinematicTestBase
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

TEST_F( WristHeuristicTest, SolveRobotWrist )
{
	auto model =
		Data::GetRevolute_Planar2R_SphericalWrist_6DOFsRobot();

	int start = 3;
	int count = 3;
	Mat4d tip_home = model->GetHomeConfiguration();
	Model::WristJointGroup wrist_group( start, count, tip_home );

	auto heuristic = Heuristic::WristHeuristic( model, wrist_group );

	VecXd seed( 6 );
	seed << M_PI / 4, M_PI / 4, M_PI / 4, 0, 0, 0;

	VecXd joints( 6 );
	joints << seed[0], seed[1], seed[2], M_PI / 4, M_PI / 4, M_PI / 4;

	auto problem = CreateProblem( model, seed, joints );
	auto result = heuristic.Presolve( problem, Solver::IKRunContext() );

	Mat4d result_pose;
	model->ComputeFK( result.joints, result_pose );

	EXPECT_TRUE( result.Sucess() );
	EXPECT_EQ( 6, result.joints.size() );
	EXPECT_TRUE( IsApprox( problem.target, result_pose ) )
	    << "target = " << std::endl << problem.target << std::endl
	    << "result = " << std::endl << result_pose << std::endl
	    << "joints = " << result.joints.transpose() << std::endl;
}

// ------------------------------------------------------------

TEST_F( WristHeuristicTest, SolveRevolute1 )
{
	// Create a joint chain with a single revolute joint
	auto single_joint_chain = Model::JointChain( 1 );
	single_joint_chain.Add(
		Model::Twist( Vec3d( 0, 0, 1 ), Vec3d( 0, 0, 0 ) ),
		Model::Link( Mat4d::Identity(), 1 ),
		Model::Limits( -M_PI, M_PI ) );

	int start = 0;
	int count = 1;
	Mat4d tip_home =
		ToTransformMatrix( Vec3d( 1, 0, 0 ) );
	Model::WristJointGroup wrist_group( start, count, tip_home );

	auto model = CreateModel( single_joint_chain, wrist_group );
	auto heuristic = Heuristic::WristHeuristic( model, wrist_group );

	VecXd seed = VecXd::Zero( 1 );
	VecXd joints( 1 );
	joints << M_PI / 4;

	auto problem = CreateProblem( model, seed, joints );
	auto result = heuristic.Presolve( problem, Solver::IKRunContext() );

	Mat4d result_pose;
	model->ComputeFK( result.joints, result_pose );

	EXPECT_TRUE( result.Sucess() );
	EXPECT_EQ( 1, result.joints.size() );
	EXPECT_TRUE( IsApprox( problem.target, result_pose ) );
}

// ------------------------------------------------------------

TEST_F( WristHeuristicTest, SolveRevolute1_Unreachable )
{
	auto single_joint_chain = Model::JointChain( 1 );
	single_joint_chain.Add(
		Model::Twist( Vec3d( 0, 0, 1 ), Vec3d( 0, 0, 0 ) ),
		Model::Link( Mat4d::Identity(), 1 ),
		Model::Limits( -M_PI, M_PI ) );

	int start = 0;
	int count = 1;
	Mat4d tip_home =
		ToTransformMatrix( Vec3d( 1, 0, 0 ) );
	Model::WristJointGroup wrist_group( start, count, tip_home );

	auto model = CreateModel( single_joint_chain, wrist_group );
	auto heuristic = Heuristic::WristHeuristic( model, wrist_group );

	VecXd seed = VecXd::Zero( 1 );
	Mat4d target;
	target << 0, 0, -1, 0,
	    0, 1,  0, 0,
	    1, 0,  0, 0,
	    0, 0,  0, 1;


	auto problem = CreateProblem( seed, target );
	auto result = heuristic.Presolve( problem, Solver::IKRunContext() );

	EXPECT_FALSE( result.Sucess() );
	EXPECT_EQ( 1, result.joints.size() );
	EXPECT_EQ( Heuristic::IKHeuristicState::PartialSuccess, result.state );
}

// ------------------------------------------------------------

TEST_F( WristHeuristicTest, SolveRevolute2 )
{
	// Create a joint chain with two revolute joints
	auto two_joint_chain = Model::JointChain( 2 );
	two_joint_chain.Add(
		Model::Twist( Vec3d( 0, 0, 1 ), Vec3d( 0, 0, 0 ) ),
		Model::Link( Mat4d::Identity(), 0 ),
		Model::Limits( -M_PI, M_PI )
		);
	two_joint_chain.Add(
		Model::Twist( Vec3d( 0, 1, 0 ), Vec3d( 0, 0, 0 ) ),
		Model::Link( Mat4d::Identity(), 1 ),
		Model::Limits( -M_PI, M_PI )
		);

	int start = 0;
	int count = 2;
	Mat4d tip_home =
		ToTransformMatrix( Vec3d( 1, 0, 0 ) );
	Model::WristJointGroup wrist_group( start, count, tip_home );

	auto model = CreateModel( two_joint_chain, wrist_group );
	auto heuristic = Heuristic::WristHeuristic( model, wrist_group );

	VecXd seed = VecXd::Zero( 2 );
	VecXd joints( 2 );
	joints << M_PI / 4, M_PI / 4;

	auto problem = CreateProblem( model, seed, joints );
	auto result = heuristic.Presolve( problem, Solver::IKRunContext() );

	Mat4d result_pose;
	model->ComputeFK( result.joints, result_pose );

	EXPECT_TRUE( result.Sucess() );
	EXPECT_EQ( 2, result.joints.size() );
	EXPECT_TRUE( IsApprox( problem.target, result_pose ) );
}

// ------------------------------------------------------------

TEST_F( WristHeuristicTest, SolveRevolute3 )
{
	auto model =
		Data::GetSphericalWristRobot();

	int start = 0;
	int count = 3;
	Mat4d tip_home =
		ToTransformMatrix( Vec3d( 0, 0, 1 ) );
	Model::WristJointGroup wrist_group( start, count, tip_home );

	auto heuristic = Heuristic::WristHeuristic( model, wrist_group );

	VecXd seed = VecXd::Zero( 3 );

	Mat4d target = Mat4d::Identity();
	target.block< 3, 3 >( 0, 0 ) = AngleAxis( M_PI / 4, Vec3d( 0, 0, 1 ) ).toRotationMatrix();
	target.block< 3, 1 >( 0, 3 ) = Vec3d( 1.0, 0.0, 0.0 );

	auto problem = CreateProblem( seed, target );
	auto result = heuristic.Presolve( problem, Solver::IKRunContext() );

	Mat4d result_pose;
	model->ComputeFK( result.joints, result_pose );

	EXPECT_TRUE( result.Sucess() );
	EXPECT_EQ( 3, result.joints.size() );
	EXPECT_TRUE( Rotation( problem.target ).isApprox( Rotation( result_pose ), rotation_tolerance ) )
	    << "target = " << std::endl << problem.target << std::endl
	    << "result = " << std::endl << result_pose << std::endl;
}

// ------------------------------------------------------------

} // namespace SOArm100::Kinematics::Test
