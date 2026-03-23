#include "Heuristic/RevoluteBaseHeuristic.hpp"

#include "Global.hpp"
#include "RobotModelTestData.hpp"
#include "KinematicTestBase.hpp"

#include "Heuristic/IKHeuristicState.hpp"
#include "Heuristic/IKPresolution.hpp"
#include "Model/JointGroup.hpp"
#include "Model/KinematicModel.hpp"
#include "Solver/IKProblem.hpp"
#include "Solver/IKRunContext.hpp"
#include "Utils/Converter.hpp"
#include "Utils/KinematicsUtils.hpp"

#include <gtest/gtest.h>
#include <cmath>
#include <ostream>

namespace SOArm100::Kinematics::Test
{

// ------------------------------------------------------------
// ------------------------------------------------------------

class BaseJointHeuristicTest : public KinematicTestBase
{
protected:
void SetUp() override
{
	model_ = Data::GetRevoluteBaseRobot();
	group_ = Model::RevoluteBaseJointGroup( model_->GetHomeConfiguration() );
}

void TearDown() override
{
}

Model::RevoluteBaseJointGroup group_{ Mat4d::Identity() };
};

// ------------------------------------------------------------
// ------------------------------------------------------------

TEST_F( BaseJointHeuristicTest, IK_Success )
{
	auto heuristic = Heuristic::RevoluteBaseHeuristic( model_, group_ );

	VecXd seed(2);
	seed << M_PI / 2, 0;

	VecXd joints( 2 );
	joints << M_PI / 4, 0;

	auto problem = CreateProblem( seed, joints );

	auto result = heuristic.Presolve( problem, Solver::IKRunContext() );

	Mat4d result_pose = ComputeFK( result.joints );

	// Check that the solution is valid
	EXPECT_EQ( result.state, Heuristic::IKHeuristicState::Success ) << "IK should succeed for reachable position";
	EXPECT_FALSE( std::isnan( result.joints[0] ) ) << "Solution should not be NaN";
	EXPECT_TRUE( IsApprox( result_pose, problem.target ) )
	    << "target= " << std::endl << problem.target << std::endl
	    << "result= " << std::endl << result_pose << std::endl;
	
	EXPECT_NEAR( M_PI / 4, result.joints[0], 1e-6 ) 
		<< "Expected joint= " << M_PI / 4 << std::endl
		<< "Result   joint= " << result.joints[0] << std::endl;
}

// ------------------------------------------------------------

TEST_F( BaseJointHeuristicTest, IK_Singularity )
{
	auto heuristic = Heuristic::RevoluteBaseHeuristic( model_, group_ );

	VecXd seed(2);
	seed << M_PI / 2, 0;

	VecXd joints( 2 );
	joints << 0, 0;

	auto problem = CreateProblem( seed, joints );
	auto result = heuristic.Presolve( problem, Solver::IKRunContext() );

	Mat4d result_pose = ComputeFK( result.joints );

	// Check that the solution is valid
	EXPECT_EQ( result.state, Heuristic::IKHeuristicState::PartialSuccess ) 
		<< "IK should partially succeed for singularity";

	EXPECT_FALSE( std::isnan( result.joints[0] ) ) 
		<< "Solution should not be NaN";

	EXPECT_TRUE( IsApprox( result_pose, problem.target ) )
	    << "target= " << std::endl << problem.target << std::endl
	    << "result= " << std::endl << result_pose << std::endl;
	
	EXPECT_EQ( seed[0], result.joints[0] ) 
		<< "Expected joint= " << seed[0] << std::endl
		<< "Result   joint= " << result.joints[0] << std::endl;
}

// ------------------------------------------------------------

TEST_F( BaseJointHeuristicTest, IK_ReferenceDirection_Singularity )
{
	// Tip Home of Revolute Base group is on the base axis
	auto singularity_tip_home = ToTransformMatrix( Vec3d( 0, 0, 1 ) );
	auto singularity_group = Model::RevoluteBaseJointGroup( singularity_tip_home );
	auto heuristic = Heuristic::RevoluteBaseHeuristic( model_, singularity_group );

	VecXd seed(2);
	seed << M_PI / 2, 0;

	VecXd joints( 2 );
	joints << 0, 0;

	auto problem = CreateProblem( seed, joints );

	auto result = heuristic.Presolve( problem, Solver::IKRunContext() );

	Mat4d result_pose = ComputeFK( result.joints );

	// Check that the solution is valid
	EXPECT_EQ( result.state, Heuristic::IKHeuristicState::Success ) << "IK should succeed for reachable position";
	EXPECT_FALSE( std::isnan( result.joints[0] ) ) << "Solution should not be NaN";
	EXPECT_TRUE( IsApprox( result_pose, problem.target ) )
	    << "target= " << std::endl << problem.target << std::endl
	    << "result= " << std::endl << result_pose << std::endl;
	
	EXPECT_NEAR( M_PI / 4, result.joints[0], 1e-6 ) 
		<< "Expected joint= " << M_PI / 4 << std::endl
		<< "Result   joint= " << result.joints[0] << std::endl;
}

// ------------------------------------------------------------

}
