#include "Heuristic/Planar1RHeuristic.hpp"
#include "Heuristic/Planar2RHeuristic.hpp"
#include "Heuristic/PlanarCCDHeuristic.hpp"

#include "Global.hpp"
#include "Heuristic/IKHeuristicState.hpp"
#include "Model/Joint/JointChainBuilder.hpp"
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
	auto single_joint_chain = CreateSimpleJointChain(
		{ RevoluteJointInfo( Vec3d::Zero(), Vec3d::UnitZ() ) }, 
		ToTransformMatrix( Vec3d( 1, 0, 0 ) ) );

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
	auto single_joint_chain = CreateSimpleJointChain(
		{ RevoluteJointInfo( Vec3d::Zero(), Vec3d::UnitZ() ) }, 
		ToTransformMatrix( Vec3d( 1, 0, 0 ) ) );

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
	auto single_joint_chain = CreateSimpleJointChain(
		{ RevoluteJointInfo( Vec3d::Zero(), Vec3d::UnitZ(), -M_PI / 4, M_PI / 4 ) }, 
		ToTransformMatrix( Vec3d( 1, 0, 0 ) ) );

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
	auto two_joint_chain = CreateSimpleJointChain(
		{ RevoluteJointInfo( Vec3d::Zero(), Vec3d::UnitZ() ), 
		 		RevoluteJointInfo( Vec3d( 0.1, 0, 0 ), Vec3d::UnitZ() ) }, 
				ToTransformMatrix( Vec3d( 0.3, 0, 0 ) ) );

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
	EXPECT_LE( result.error, problem.tolerance );
	EXPECT_LE( TranslationError( problem.target, result_pose ), problem.tolerance )
	    << problem << std::endl
	    << "Target = \n" << Translation( problem.target ) << std::endl
	    << "Result = \n" << Translation( result_pose ) << std::endl;
}

// ------------------------------------------------------------

TEST_F( Planar2RHeuristicTest, SolveSuccessBothSolutionWithinLimits )
{
	auto two_joint_chain = CreateSimpleJointChain(
		{ RevoluteJointInfo( Vec3d( 0, 0, 0.5  ), Vec3d::UnitY() ), 
		 		RevoluteJointInfo( Vec3d( 0., 0., 1.0 ), Vec3d::UnitY() ) }, 
				ToTransformMatrix( Vec3d( 0.6, 0, 1.0 ) ) );

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
	EXPECT_LE( result1.error, problem.tolerance );
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
	EXPECT_LE( result2.error, problem.tolerance );
	EXPECT_LE( TranslationError( problem.target, result_pose ), problem.tolerance )
	    << problem << std::endl
	    << "Target = \n" << Translation( problem.target ) << std::endl
	    << "Result = \n" << Translation( result_pose ) << std::endl;

	EXPECT_NE( result1.joints, result2.joints );
}

// ------------------------------------------------------------

TEST_F( Planar2RHeuristicTest, ChooseClosestElbowConfiguration )
{
	auto two_joint_chain = CreateSimpleJointChain(
		{ RevoluteJointInfo( Vec3d::Zero(), Vec3d::UnitZ() ), 
		 		RevoluteJointInfo( Vec3d( 0.1, 0, 0 ), Vec3d::UnitZ() ) }, 
				ToTransformMatrix( Vec3d( 0.3, 0, 0 ) ) );

	int start = 0;
	int count = 2;
	Mat4d tip_home = ToTransformMatrix( Vec3d( 2, 0, 0 ) );
	Model::PlanarNRJointGroup planar_group( start, count, tip_home );

	auto model = CreateModel( two_joint_chain, planar_group );
	auto heuristic = Heuristic::Planar2RHeuristic( model, planar_group );

	// Seed biased heavily toward an elbow-down configuration (negative elbow angle)
	VecXd seed( 2 );
	seed << 0.5, -M_PI;

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
	auto two_joint_chain = CreateSimpleJointChain(
		{ RevoluteJointInfo( Vec3d::Zero(), Vec3d::UnitZ() ), 
		 		RevoluteJointInfo( Vec3d( 0.1, 0, 0 ), Vec3d::UnitZ() ) }, 
				ToTransformMatrix( Vec3d( 0.3, 0, 0 ) ) );

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
	auto two_joint_chain = CreateSimpleJointChain(
		{ RevoluteJointInfo( Vec3d::Zero(), Vec3d::UnitZ() ), 
		 		RevoluteJointInfo( Vec3d( 0.1, 0, 0 ), Vec3d::UnitZ(), 0, M_PI ) }, 
				ToTransformMatrix( Vec3d( 0.3, 0, 0 ) ) );
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
	// Create Problem
	VecXd seed( 2 );
	seed << 0.01, 0.01;
	VecXd joints( 2 );
	joints << M_PI / 3, M_PI / 3; // Target requires a solution far outside allowed boundaries
	Mat4d tip_home = ToTransformMatrix( Vec3d( 0.3, 0, 0 ) );
	auto two_joint_chain = CreateSimpleJointChain(
		{ RevoluteJointInfo( Vec3d::Zero(), Vec3d::UnitZ() ), 
		 		RevoluteJointInfo( Vec3d( 0.1, 0, 0 ), Vec3d::UnitZ() ) }, 
				tip_home );
	Mat4d target;
	two_joint_chain->ComputeFK( joints, tip_home, target );
	auto problem = CreateProblem( seed, target );

	auto two_joint_chain_tight = CreateSimpleJointChain(
		{ RevoluteJointInfo( Vec3d::Zero(), Vec3d::UnitZ(), -M_PI / 12, M_PI / 12 ), 
		 		RevoluteJointInfo( Vec3d( 0.1, 0, 0 ), Vec3d::UnitZ(), -M_PI / 12, M_PI / 12 ) }, 
				tip_home );

	int start = 0;
	int count = 2;
	Model::PlanarNRJointGroup planar_group( start, count, tip_home );

	auto model = CreateModel( two_joint_chain_tight, planar_group );
	auto heuristic = Heuristic::Planar2RHeuristic( model, planar_group );

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
	Mat4d tip_home = ToTransformMatrix( Vec3d( 3, 0, 0 ) );

	auto three_joint_chain = CreateSimpleJointChain(
		{ RevoluteJointInfo( Vec3d( 0, 0, 0 ), Vec3d::UnitZ() ), 
		 		RevoluteJointInfo( Vec3d( 1, 0, 0 ), Vec3d::UnitZ() ) , 
		 		RevoluteJointInfo( Vec3d( 2, 0, 0 ), Vec3d::UnitZ() ) }, 
				tip_home );

	int start = 0;
	int count = 3;
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
	Mat4d tip_home = ToTransformMatrix( Vec3d( 3, 0, 0 ) );

	auto three_joint_chain = CreateSimpleJointChain(
		{ RevoluteJointInfo( Vec3d::Zero(), Vec3d::UnitZ() ), 
		 		RevoluteJointInfo( Vec3d( 1, 0, 0 ), Vec3d::UnitZ() ) , 
		 		RevoluteJointInfo( Vec3d( 2, 0, 0 ), Vec3d::UnitZ() ) }, 
				tip_home );

	int start = 0;
	int count = 3;
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
	Mat4d tip_home = ToTransformMatrix( Vec3d( 3, 0, 0 ) );

	auto three_joint_chain = CreateSimpleJointChain(
		{ RevoluteJointInfo( Vec3d::Zero(), Vec3d::UnitZ() ), 
		 		RevoluteJointInfo( Vec3d( 1, 0, 0 ), Vec3d::UnitZ() ) , 
		 		RevoluteJointInfo( Vec3d( 2, 0, 0 ), Vec3d::UnitZ() ) }, 
				tip_home );

	int start = 0;
	int count = 3;
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
	Mat4d tip_home = ToTransformMatrix( Vec3d( 3, 0, 0 ) );

	auto three_joint_chain = CreateSimpleJointChain(
		{ RevoluteJointInfo( Vec3d::Zero(), Vec3d::UnitZ(), -M_PI / 12, M_PI / 12 ), 
		 		RevoluteJointInfo( Vec3d( 1, 0, 0 ), Vec3d::UnitZ(), -M_PI / 12, M_PI / 12 ) , 
		 		RevoluteJointInfo( Vec3d( 2, 0, 0 ), Vec3d::UnitZ(), -M_PI / 12, M_PI / 12 ) }, 
				tip_home );

	int start = 0;
	int count = 3;
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