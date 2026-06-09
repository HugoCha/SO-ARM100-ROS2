#include "Heuristic/Planar1RHeuristic.hpp"
#include "Heuristic/Planar2RHeuristic.hpp"
#include "Heuristic/PlanarCCDHeuristic.hpp"

#include "Global.hpp"
#include "Heuristic/IKHeuristicState.hpp"
#include "RobotModelTestData.hpp"
#include "Solver/IKRunContext.hpp"
#include "KinematicTestBase.hpp"

#include "Heuristic/IKPresolution.hpp"
#include "Model/Joint/JointChain.hpp"
#include "Model/Joint/JointGroup.hpp"
#include "Model/KinematicModel.hpp"
#include "Utils/Converter.hpp"
#include "Utils/StringConverter.hpp"
#include "Utils/KinematicsUtils.hpp"

#include <gtest/gtest.h>
#include <cmath>
#include <ostream>

namespace SOArm100::Kinematics::Test
{

// ------------------------------------------------------------
// ------------------------------------------------------------

class Planar1RHeuristicTest : public KinematicTestBase
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
// Planar 1R Heuristic
// ------------------------------------------------------------

TEST_F( Planar1RHeuristicTest, SolveSuccessWithinLimits )
{
	// Create a joint chain with a single active revolute joint along the Z axis
	auto single_joint_chain = Model::JointChain( 1 );
	single_joint_chain.Add( "",
	                        Model::Twist( Vec3d( 0, 0, 1 ), Vec3d( 0, 0, 0 ) ),
	                        Model::Link( "", Mat4d::Identity(), 1.0 ), // Link length = 1.0
	                        Model::Limits( -M_PI, M_PI ) );

	int start = 0;
	int count = 1;
	Mat4d tip_home = ToTransformMatrix( Vec3d( 1, 0, 0 ) );
	Model::PlanarNRJointGroup planar_group( start, count, tip_home );

	auto model = CreateModel( single_joint_chain, planar_group );
	auto heuristic = Heuristic::Planar1RHeuristic( model, planar_group );

	VecXd seed = VecXd::Zero( 1 );
	VecXd joints( 1 );
	joints << M_PI / 4; // Target configuration at 45 degrees

	auto problem = CreateProblem( model, seed, joints );
	auto result = heuristic.Presolve( problem, Solver::IKRunContext() );

	Mat4d result_pose;
	model->ComputeFK( result.joints, result_pose );

	EXPECT_EQ( Heuristic::IKHeuristicState::Success, result.state );
	EXPECT_EQ( 1, result.joints.size() );
	EXPECT_NEAR( M_PI / 4, result.joints[0], 1e-6 );
	EXPECT_LE( TranslationError( problem.target, result_pose ), problem.tolerance );
}

// ------------------------------------------------------------

TEST_F( Planar1RHeuristicTest, SolveUnreachableDistance )
{
	auto single_joint_chain = Model::JointChain( 1 );
	single_joint_chain.Add( "",
	                        Model::Twist( Vec3d( 0, 0, 1 ), Vec3d( 0, 0, 0 ) ),
	                        Model::Link( "", Mat4d::Identity(), 1.0 ), // Link length = 1.0
	                        Model::Limits( -M_PI, M_PI ) );

	int start = 0;
	int count = 1;
	Mat4d tip_home = ToTransformMatrix( Vec3d( 1, 0, 0 ) );
	Model::PlanarNRJointGroup planar_group( start, count, tip_home );

	auto model = CreateModel( single_joint_chain, planar_group );
	auto heuristic = Heuristic::Planar1RHeuristic( model, planar_group );

	VecXd seed = VecXd::Zero( 1 );

	// Target position is physically further away (D = 1.5) than link length (L = 1.0)
	Mat4d target = ToTransformMatrix( Vec3d( 1.5, 0, 0 ) );

	auto problem = CreateProblem( seed, target );
	auto result = heuristic.Presolve( problem, Solver::IKRunContext() );

	EXPECT_EQ( Heuristic::IKHeuristicState::Fail, result.state );
	EXPECT_EQ( 1, result.joints.size() );
	EXPECT_EQ( seed, result.joints );
}

// ------------------------------------------------------------

TEST_F( Planar1RHeuristicTest, SolveExceedsJointLimits_PartialSuccessOrFail )
{
	auto single_joint_chain = Model::JointChain( 1 );
	single_joint_chain.Add( "",
	                        Model::Twist( Vec3d( 0, 0, 1 ), Vec3d( 0, 0, 0 ) ),
	                        Model::Link( "", Mat4d::Identity(), 1.0 ),
	                        Model::Limits( -M_PI / 4, M_PI / 4 ) ); // Restricted limits (-45 to 45 deg)

	int start = 0;
	int count = 1;
	Mat4d tip_home = ToTransformMatrix( Vec3d( 1, 0, 0 ) );
	Model::PlanarNRJointGroup planar_group( start, count, tip_home );

	auto model = CreateModel( single_joint_chain, planar_group );
	auto heuristic = Heuristic::Planar1RHeuristic( model, planar_group );

	VecXd seed = VecXd::Zero( 1 );
	Mat4d target = ToTransformMatrix( Vec3d( 0, 1, 0 ) );

	auto problem = CreateProblem( seed, target );
	auto result = heuristic.Presolve( problem, Solver::IKRunContext() );

	EXPECT_NE( Heuristic::IKHeuristicState::Success, result.state );
	EXPECT_EQ( 1, result.joints.size() );
	// Value must be clamped explicitly to the upper limit (45 deg)
	EXPECT_NEAR( M_PI / 4, result.joints[0], 1e-6 );
}

// ------------------------------------------------------------
// Planar 2R Heuristic
// ------------------------------------------------------------

class Planar2RHeuristicTest : public KinematicTestBase
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

TEST_F( Planar2RHeuristicTest, SolveSuccessWithinLimits )
{
	// Create a 2R chain in the XY plane (rotation about Z)
	auto two_joint_chain = Model::JointChain( 2 );
	two_joint_chain.Add( "",
	                     Model::Twist( Vec3d( 0, 0, 1 ), Vec3d( 0, 0, 0 ) ),
	                     Model::Link( "", Mat4d::Identity(), 0.1 ), // L1 = 1.0
	                     Model::Limits( -M_PI, M_PI ) );
	two_joint_chain.Add( "",
	                     Model::Twist( Vec3d( 0, 0, 1 ), Vec3d( 0.1, 0, 0 ) ),
	                     Model::Link( "", ToTransformMatrix( Vec3d( 0.1, 0, 0 ) ), 0.2 ), // L2 = 1.0
	                     Model::Limits( -M_PI, M_PI ) );

	int start = 0;
	int count = 2;
	Mat4d tip_home = ToTransformMatrix( Vec3d( 0.3, 0, 0 ) );
	Model::PlanarNRJointGroup planar_group( start, count, tip_home );

	auto model = CreateModel( two_joint_chain, planar_group );
	auto heuristic = Heuristic::Planar2RHeuristic( model, planar_group );

	VecXd seed = VecXd::Zero( 2 );
	VecXd joints( 2 );
	joints << M_PI / 6, -M_PI / 3; // Target pose configurations

	auto problem = CreateProblem( model, seed, joints );
	auto result = heuristic.Presolve( problem, Solver::IKRunContext() );

	Mat4d result_pose;
	model->ComputeFK( result.joints, result_pose );

	EXPECT_EQ( Heuristic::IKHeuristicState::Success, result.state );
	EXPECT_EQ( 2, result.joints.size() );
	EXPECT_LE( TranslationError( problem.target, result_pose ), problem.tolerance )
	    << problem << std::endl
	    << "Target = \n" << Translation( problem.target ) << std::endl
	    << "Result = \n" << Translation( result_pose ) << std::endl;
}

// ------------------------------------------------------------

TEST_F( Planar2RHeuristicTest, SolveSuccessBothSolutionWithinLimits )
{
	// Create a 2R chain in the XY plane (rotation about Z)
	auto two_joint_chain = Model::JointChain( 2 );
	two_joint_chain.Add( "",
	                     Model::Twist( Vec3d( 0, 1, 0 ), Vec3d( 0, 0, 0.5 ) ),
	                     Model::Link( "", ToTransformMatrix( Vec3d( 0, 0, 0.5 ) ), 0.5 ), // L1 = 1.0
	                     Model::Limits( -M_PI, M_PI ) );
	two_joint_chain.Add( "",
	                     Model::Twist( Vec3d( 0, 1, 0 ), Vec3d( 0., 0., 1.0 ) ),
	                     Model::Link( "", ToTransformMatrix( Vec3d( 0., 0., 1.0 ) ), 0.6 ), // L2 = 1.0
	                     Model::Limits( -M_PI, M_PI ) );

	int start = 0;
	int count = 2;
	Mat4d tip_home = ToTransformMatrix( Vec3d( 0.6, 0.0, 1.0 ) );
	Model::PlanarNRJointGroup planar_group( start, count, tip_home );

	auto model = CreateModel( two_joint_chain, tip_home );
	auto heuristic = Heuristic::Planar2RHeuristic( model, planar_group );

	VecXd seed( 2 );
	seed[0] = -M_PI;
	seed[1] = 0;
	VecXd joints( 2 );
	joints << -M_PI / 2, 0; // Target pose configurations

	auto problem = CreateProblem( model, seed, joints );
	auto result1 = heuristic.Presolve( problem, Solver::IKRunContext() );

	Mat4d result_pose;
	model->ComputeFK( result1.joints, result_pose );
	EXPECT_EQ( Heuristic::IKHeuristicState::Success, result1.state );
	EXPECT_EQ( 2, result1.joints.size() );
	EXPECT_LE( TranslationError( problem.target, result_pose ), problem.tolerance  )
	    << problem << std::endl
	    << "Target = \n" << Translation( problem.target ) << std::endl
	    << "Result = \n" << Translation( result_pose ) << std::endl;

	seed[0] = M_PI;
	seed[1] = -M_PI;
	problem = CreateProblem( model, seed, joints );
	auto result2 = heuristic.Presolve( problem, Solver::IKRunContext() );
	result_pose = ComputeFK( model, result2.joints );
	EXPECT_EQ( Heuristic::IKHeuristicState::Success, result2.state );
	EXPECT_EQ( 2, result2.joints.size() );
	EXPECT_LE( TranslationError( problem.target, result_pose ), problem.tolerance )
	    << problem << std::endl
	    << "Target = \n" << Translation( problem.target ) << std::endl
	    << "Result = \n" << Translation( result_pose ) << std::endl;

	EXPECT_NE( result1.joints, result2.joints );
}

// ------------------------------------------------------------

TEST_F( Planar2RHeuristicTest, ChooseClosestElbowConfiguration )
{
	auto two_joint_chain = Model::JointChain( 2 );
	two_joint_chain.Add( "",
	                     Model::Twist( Vec3d( 0, 0, 1 ), Vec3d( 0, 0, 0 ) ),
	                     Model::Link( "", Mat4d::Identity(), 1.0 ),
	                     Model::Limits( -M_PI, M_PI ) );
	two_joint_chain.Add( "",
	                     Model::Twist( Vec3d( 0, 0, 1 ), Vec3d( 1, 0, 0 ) ),
	                     Model::Link( "", ToTransformMatrix( Vec3d( 1, 0, 0 ) ), 1.0 ), // L2 = 1.0
	                     Model::Limits( -M_PI, M_PI ) );

	int start = 0;
	int count = 2;
	Mat4d tip_home = ToTransformMatrix( Vec3d( 2, 0, 0 ) );
	Model::PlanarNRJointGroup planar_group( start, count, tip_home );

	auto model = CreateModel( two_joint_chain, planar_group );
	auto heuristic = Heuristic::Planar2RHeuristic( model, planar_group );

	// Seed biased heavily toward an elbow-down configuration (negative elbow angle)
	VecXd seed( 2 );
	seed << 0.5, -0.5;

	VecXd joints( 2 );
	joints << M_PI / 6, M_PI / 3; // Forward kinematics target (has two solutions)

	auto problem = CreateProblem( model, seed, joints );
	auto result = heuristic.Presolve( problem, Solver::IKRunContext() );

	Mat4d result_pose;
	model->ComputeFK( result.joints, result_pose );

	EXPECT_EQ( Heuristic::IKHeuristicState::Success, result.state );
	EXPECT_TRUE( Translation( problem.target ).isApprox( Translation( result_pose ) ) )
	    << "Target = \n" << Translation( problem.target ) << std::endl
	    << "Result = \n" << Translation( result_pose ) << std::endl;
	// Ensure the solver picked the elbow-down configuration because of the seed bias
	EXPECT_LT( result.joints[1], 0.0 );
}

// ------------------------------------------------------------

TEST_F( Planar2RHeuristicTest, SolveUnreachableDistance )
{
	auto two_joint_chain = Model::JointChain( 2 );
	two_joint_chain.Add( "",
	                     Model::Twist( Vec3d( 0, 0, 1 ), Vec3d( 0, 0, 0 ) ),
	                     Model::Link( "", Mat4d::Identity(), 1.0 ), // L1 = 1.0
	                     Model::Limits( -M_PI, M_PI ) );
	two_joint_chain.Add( "",
	                     Model::Twist( Vec3d( 0, 0, 1 ), Vec3d( 1, 0, 0 ) ),
	                     Model::Link( "", ToTransformMatrix( Vec3d( 1, 0, 0 ) ), 1.0 ), // L2 = 1.0
	                     Model::Limits( -M_PI, M_PI ) );

	int start = 0;
	int count = 2;
	Mat4d tip_home = ToTransformMatrix( Vec3d( 2, 0, 0 ) );
	Model::PlanarNRJointGroup planar_group( start, count, tip_home );

	auto model = CreateModel( two_joint_chain, planar_group );
	auto heuristic = Heuristic::Planar2RHeuristic( model, planar_group );

	VecXd seed = VecXd::Zero( 2 );

	// Total reach length L1 + L2 = 2.0. Target is placed out of bounds at 2.5
	Mat4d target = ToTransformMatrix( Vec3d( 2.5, 0, 0 ) );

	auto problem = CreateProblem( seed, target );
	auto result = heuristic.Presolve( problem, Solver::IKRunContext() );

	EXPECT_EQ( Heuristic::IKHeuristicState::Fail, result.state );
	EXPECT_EQ( 2, result.joints.size() );
	EXPECT_EQ( seed, result.joints );
}

// ------------------------------------------------------------

TEST_F( Planar2RHeuristicTest, JointLimitsForceSingleValidConfiguration )
{
	auto two_joint_chain = Model::JointChain( 2 );
	two_joint_chain.Add( "",
	                     Model::Twist( Vec3d( 0, 0, 1 ), Vec3d( 0, 0, 0 ) ),
	                     Model::Link( "", Mat4d::Identity(), 1.0 ),
	                     Model::Limits( -M_PI, M_PI ) );
	two_joint_chain.Add( "",
	                     Model::Twist( Vec3d( 0, 0, 1 ), Vec3d( 1, 0, 0 ) ),
	                     Model::Link( "", ToTransformMatrix( Vec3d( 1, 0, 0 ) ), 1.0 ), // L2 = 1.0
	                     Model::Limits( 0.0, M_PI ) );
	int start = 0;
	int count = 2;
	Mat4d tip_home = ToTransformMatrix( Vec3d( 2, 0, 0 ) );
	Model::PlanarNRJointGroup planar_group( start, count, tip_home );

	auto model = CreateModel( two_joint_chain, planar_group );
	auto heuristic = Heuristic::Planar2RHeuristic( model, planar_group );

	// Seed tries to force elbow down config, but limits should override it
	VecXd seed( 2 );
	seed << 0.5, -0.2;

	VecXd joints( 2 );
	joints << M_PI / 4, M_PI / 4;

	auto problem = CreateProblem( model, seed, joints );
	auto result = heuristic.Presolve( problem, Solver::IKRunContext() );

	EXPECT_EQ( Heuristic::IKHeuristicState::Success, result.state );
	EXPECT_GE( result.joints[1], 0.0 ); // Must select elbow-up variant due to limits
}

// ------------------------------------------------------------

TEST_F( Planar2RHeuristicTest, BothConfigurationsInvalid_PartialSuccess )
{
	auto two_joint_chain = Model::JointChain( 2 );
	two_joint_chain.Add( "",
	                     Model::Twist( Vec3d( 0, 0, 1 ), Vec3d( 0, 0, 0 ) ),
	                     Model::Link( "", Mat4d::Identity(), 1.0 ),
	                     Model::Limits( -M_PI / 12, M_PI / 12 ) ); // Highly restricted shoulder limits
	two_joint_chain.Add( "",
	                     Model::Twist( Vec3d( 0, 0, 1 ), Vec3d( 1, 0, 0 ) ),
	                     Model::Link( "", ToTransformMatrix( Vec3d( 1, 0, 0 ) ), 1.0 ), // L2 = 1.0
	                     Model::Limits( -M_PI / 12, M_PI / 12 ) );

	int start = 0;
	int count = 2;
	Mat4d tip_home = ToTransformMatrix( Vec3d( 2, 0, 0 ) );
	Model::PlanarNRJointGroup planar_group( start, count, tip_home );

	auto model = CreateModel( two_joint_chain, planar_group );
	auto heuristic = Heuristic::Planar2RHeuristic( model, planar_group );

	VecXd seed( 2 );
	seed << 0.01, 0.01;

	VecXd joints( 2 );
	joints << M_PI / 3, M_PI / 3; // Target requires a solution far outside allowed boundaries

	auto problem = CreateProblem( model, seed, joints );
	auto result = heuristic.Presolve( problem, Solver::IKRunContext() );

	// Expecting code fallback pathway where both configurations fail validation checks
	EXPECT_EQ( Heuristic::IKHeuristicState::Fail, result.state );
}

// ------------------------------------------------------------
// Planar CCD Heuristic
// ------------------------------------------------------------

class PlanarCCDHeuristicTest : public KinematicTestBase
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

TEST_F( PlanarCCDHeuristicTest, SolveSuccessWithinTolerance )
{
	// Create a 3R planar joint chain (3 degrees of freedom)
	auto three_joint_chain = Model::JointChain( 3 );
	three_joint_chain.Add( "",
	                       Model::Twist( Vec3d( 0, 0, 1 ), Vec3d( 0, 0, 0 ) ),
	                       Model::Link( "", Mat4d::Identity(), 1.0 ),
	                       Model::Limits( -M_PI, M_PI ) );
	three_joint_chain.Add( "",
	                       Model::Twist( Vec3d( 0, 0, 1 ), Vec3d( 1, 0, 0 ) ),
	                       Model::Link( "", ToTransformMatrix( Vec3d( 1, 0, 0 ) ), 1.0 ),
	                       Model::Limits( -M_PI, M_PI ) );
	three_joint_chain.Add( "",
	                       Model::Twist( Vec3d( 0, 0, 1 ), Vec3d( 2, 0, 0 ) ),
	                       Model::Link( "", ToTransformMatrix( Vec3d( 2, 0, 0 ) ), 1.0 ),
	                       Model::Limits( -M_PI, M_PI ) );

	int start = 0;
	int count = 3;
	Mat4d tip_home = ToTransformMatrix( Vec3d( 3, 0, 0 ) );
	Model::PlanarNRJointGroup planar_group( start, count, tip_home );

	auto model = CreateModel( three_joint_chain, planar_group );

	// Setup typical configuration solver parameters
	Heuristic::PlanarCCDHeuristic::SolverParameters params;
	params.max_iterations = 1000;
	params.max_stalled_iterations = 100;

	auto heuristic = Heuristic::PlanarCCDHeuristic( model, planar_group, params );

	VecXd seed = VecXd::Zero( 3 );
	VecXd joints( 3 );
	joints << M_PI / 6, -M_PI / 6, M_PI / 6; // Reach target position configuration

	auto problem = CreateProblem( model, seed, joints );
	auto result = heuristic.Presolve( problem, Solver::IKRunContext() );

	Mat4d result_pose;
	model->ComputeFK( result.joints, result_pose );

	EXPECT_EQ( Heuristic::IKHeuristicState::Success, result.state );
	EXPECT_EQ( 3, result.joints.size() );
	EXPECT_LT( result.error, problem.tolerance );
	EXPECT_LE( result.iterations, params.max_iterations );
	EXPECT_TRUE( Translation( problem.target ).isApprox( Translation( result_pose ), problem.tolerance ) )
	    << "Target = \n" << Translation( problem.target ) << std::endl
	    << "Result = \n" << Translation( result_pose ) << std::endl;
}

// ------------------------------------------------------------

TEST_F( PlanarCCDHeuristicTest, CheckConsistency_ValidConfiguration )
{
	// Create a 3R planar joint chain (3 degrees of freedom)
	auto three_joint_chain = Model::JointChain( 3 );
	Vec3d origin = Vec3d( 0, 0.2, 0.2 );;
	Vec3d next_origin = Vec3d( 0, 0.2, 0.6 );
	Vec3d axis = Vec3d::UnitY();
	three_joint_chain.Add( "",
	                       Model::Twist( axis, origin ),
	                       Model::Link( "", ToTransformMatrix( origin ), ToTransformMatrix( next_origin ) ),
	                       Model::Limits( -M_PI, M_PI ) );
	origin = next_origin;
	next_origin = Vec3d( 0.4, 0, 0.6 );
	axis = Vec3d::UnitY();
	three_joint_chain.Add( "",
	                       Model::Twist( axis, origin ),
	                       Model::Link( "", ToTransformMatrix( origin ), ToTransformMatrix( next_origin ) ),
	                       Model::Limits( -M_PI, M_PI ) );
	origin = next_origin;
	next_origin = Vec3d( 0.4, 0.2, 0.6 );
	axis = Vec3d::UnitY();
	three_joint_chain.Add( "",
	                       Model::Twist( axis, origin ),
	                       Model::Link( "", ToTransformMatrix( origin ), ToTransformMatrix( next_origin ) ),
	                       Model::Limits( -M_PI, M_PI ) );

	int start = 0;
	int count = 3;
	Mat4d tip_home = ToTransformMatrix( Vec3d( 0.5, 0.2, 0.6 ) );
	Model::PlanarNRJointGroup planar_group( start, count, tip_home );

	auto model = CreateModel( three_joint_chain, planar_group );

	// Setup typical configuration solver parameters
	Heuristic::PlanarCCDHeuristic::SolverParameters params;
	params.max_iterations = 200;
	params.max_stalled_iterations = 50;

	auto heuristic = Heuristic::PlanarCCDHeuristic( model, planar_group, params );

	const int ITER = 1;
	int k_successes = 0;
	double avg_iteration = 0.0;
	double avg_error = 0.0;
	random_numbers::RandomNumberGenerator rng;
	double tolerance = 5e-3;
	VecXd seed = VecXd::Zero( 3 );
	VecXd joints( 3 );

	for ( int i = 0; i < ITER; i++ )
	{
		joints = model->GetChain()->RandomValidJoints( rng );
		seed = model->GetChain()->RandomValidJointsNear( rng, joints, 0.3 );
		// Mat4d target;
		// target <<
		// -0.317482 ,        0 , 0.948264 ,-0.273223
		// ,        0,         1,         0,       0.2
		// ,-0.948264,         0, -0.317482, -0.655129
		// ,        0,         0,         0,         1;

		// auto problem = CreateProblem( seed, target, tolerance );
		auto problem = CreateProblem( model, seed, joints, tolerance );
		auto result = heuristic.Presolve( problem, Solver::IKRunContext() );

		if ( !result.Success() )
		{
			Mat4d result_pose;
			model->ComputeFK( result.joints, result_pose );

			std::cout << "Initial joints = " << joints.transpose() << std::endl;
			std::cout << problem << std::endl;
			std::cout << result << std::endl;
			std::cout << "Result = \n" << Translation( result_pose ) << std::endl;
		}
		else
		{
			k_successes++;
		}

		avg_iteration += result.iterations / ( double )ITER;
		avg_error += result.error / ( double )ITER;
	}

	EXPECT_GE( k_successes, 0.99 * ITER );
	EXPECT_LE( avg_iteration, 20 );
	EXPECT_LE( avg_error, tolerance );
}

// ------------------------------------------------------------

TEST_F( PlanarCCDHeuristicTest, ExceedsIterations_ReturnsPartialSuccess )
{
	auto three_joint_chain = Model::JointChain( 3 );
	three_joint_chain.Add( "",
	                       Model::Twist( Vec3d( 0, 0, 1 ), Vec3d( 0, 0, 0 ) ),
	                       Model::Link( "", Mat4d::Identity(), 1.0 ),
	                       Model::Limits( -M_PI, M_PI ) );
	three_joint_chain.Add( "",
	                       Model::Twist( Vec3d( 0, 0, 1 ), Vec3d( 1, 0, 0 ) ),
	                       Model::Link( "", ToTransformMatrix( Vec3d( 1, 0, 0 ) ), 1.0 ),
	                       Model::Limits( -M_PI, M_PI ) );
	three_joint_chain.Add( "",
	                       Model::Twist( Vec3d( 0, 0, 1 ), Vec3d( 2, 0, 0 ) ),
	                       Model::Link( "", ToTransformMatrix( Vec3d( 2, 0, 0 ) ), 1.0 ),
	                       Model::Limits( -M_PI, M_PI ) );

	int start = 0;
	int count = 3;
	Mat4d tip_home = ToTransformMatrix( Vec3d( 3, 0, 0 ) );
	Model::PlanarNRJointGroup planar_group( start, count, tip_home );

	auto model = CreateModel( three_joint_chain, planar_group );

	// Force failure path via strict limits on iterations
	Heuristic::PlanarCCDHeuristic::SolverParameters params;
	params.max_iterations = 2;

	auto heuristic = Heuristic::PlanarCCDHeuristic( model, planar_group, params );

	VecXd seed = VecXd::Zero( 3 );
	VecXd joints( 3 );
	joints << M_PI / 2, M_PI / 4, M_PI / 4; // Target quite far from seed zero configuration

	auto problem = CreateProblem( model, seed, joints );
	problem.tolerance = 1e-6;
	auto result = heuristic.Presolve( problem, Solver::IKRunContext() );

	// Should break loop early and fallback gracefully to partial success
	EXPECT_EQ( Heuristic::IKHeuristicState::PartialSuccess, result.state );
	EXPECT_EQ( params.max_iterations, result.iterations );
	EXPECT_GE( result.error, problem.tolerance );
}

// ------------------------------------------------------------

TEST_F( PlanarCCDHeuristicTest, JointLimitsClampingEnforced )
{
	auto three_joint_chain = Model::JointChain( 3 );
	three_joint_chain.Add( "",
	                       Model::Twist( Vec3d( 0, 0, 1 ), Vec3d( 0, 0, 0 ) ),
	                       Model::Link( "", Mat4d::Identity(), 1.0 ),
	                       Model::Limits( -M_PI / 12, M_PI / 12 ) );
	three_joint_chain.Add( "",
	                       Model::Twist( Vec3d( 0, 0, 1 ), Vec3d( 1, 0, 0 ) ),
	                       Model::Link( "", ToTransformMatrix( Vec3d( 1, 0, 0 ) ), 1.0 ),
	                       Model::Limits( -M_PI / 12, M_PI / 12 ) );
	three_joint_chain.Add( "",
	                       Model::Twist( Vec3d( 0, 0, 1 ), Vec3d( 2, 0, 0 ) ),
	                       Model::Link( "", ToTransformMatrix( Vec3d( 2, 0, 0 ) ), 1.0 ),
	                       Model::Limits( -M_PI / 12, M_PI / 12 ) );

	int start = 0;
	int count = 3;
	Mat4d tip_home = ToTransformMatrix( Vec3d( 3, 0, 0 ) );
	Model::PlanarNRJointGroup planar_group( start, count, tip_home );

	auto model = CreateModel( three_joint_chain, planar_group );

	Heuristic::PlanarCCDHeuristic::SolverParameters params;
	params.max_iterations = 20;

	auto heuristic = Heuristic::PlanarCCDHeuristic( model, planar_group, params );

	VecXd seed = VecXd::Zero( 3 );

	// Explicit out-of-reach target forcing joint rotation configurations up against limits
	Mat4d target = ToTransformMatrix( Vec3d( 2.9, 1.0, 0.0 ) );

	auto problem = CreateProblem( seed, target, 1e-4 );
	auto result = heuristic.Presolve( problem, Solver::IKRunContext() );

	// Due to system constraints, joints must remain clamped inside strict bounds
	for ( int i = 0; i < result.joints.size(); ++i )
	{
		EXPECT_GE( result.joints[i], -M_PI / 12 - 1e-6 );
		EXPECT_LE( result.joints[i],  M_PI / 12 + 1e-6 );
	}
}

// ------------------------------------------------------------

} // namespace SOArm100::Kinematics::Test