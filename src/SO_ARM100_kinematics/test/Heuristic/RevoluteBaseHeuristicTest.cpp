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

class RevoluteBaseHeuristicTest : public KinematicTestBase
{
protected:
void SetUp() override
{
	model_ = Data::GetRevoluteBaseRobot();
	revolute_base_group_ = *model_->GetTopology().Get( Model::revolute_base_name );
}

void TearDown() override
{
}

Model::JointGroup revolute_base_group_;
};

// ------------------------------------------------------------
// ------------------------------------------------------------

TEST_F( RevoluteBaseHeuristicTest, IK_Success )
{
	auto heuristic = Heuristic::RevoluteBaseHeuristic( model_, revolute_base_group_ );

	VecXd seed(3);
	seed << M_PI / 2, 0, 0;

	VecXd joints( 3 );
	joints << M_PI / 4, 0, 0;

	auto problem = CreateProblem( seed, joints );
	auto result = heuristic.Presolve( problem, Solver::IKRunContext() );

	Mat4d result_pose = ComputeFK( result.joints );

	// Check that the solution is valid
	EXPECT_EQ( result.state, Heuristic::IKHeuristicState::Success ) << "IK should succeed for reachable position";
	EXPECT_FALSE( std::isnan( result.joints[0] ) ) << "Solution should not be NaN";
	EXPECT_LE( TranslationError( result_pose, problem.target ), epsilon )
	    << "target= " << std::endl << problem.target << std::endl
	    << "result= " << std::endl << result_pose << std::endl;
	
	EXPECT_NEAR( M_PI / 4, result.joints[0], 1e-6 ) 
		<< "Expected joint= " << M_PI / 4 << std::endl
		<< "Result   joint= " << result.joints[0] << std::endl;
}

// ------------------------------------------------------------

TEST_F( RevoluteBaseHeuristicTest, IK_Singularity )
{
	auto heuristic = Heuristic::RevoluteBaseHeuristic( model_, revolute_base_group_ );

	VecXd seed( 3 );
	seed << 0, -M_PI / 2, 0;

	VecXd joints(3);
	joints << M_PI / 2, -M_PI / 2, 0;

	auto problem = CreateProblem( seed, joints );
	auto result = heuristic.Presolve( problem, Solver::IKRunContext() );

	EXPECT_EQ( result.state, Heuristic::IKHeuristicState::PartialSuccess ) 
		<< "IK should partially succeed for singularity";

	EXPECT_EQ( seed[0], result.joints[0] ) 
		<< "Expected joint= " << seed[0] << std::endl
		<< "Result   joint= " << result.joints[0] << std::endl;
}

// ------------------------------------------------------------

TEST_F( RevoluteBaseHeuristicTest, IK_ReferenceDirection_Singularity )
{
	// Tip Home of Revolute Base group is on the base axis
	auto singularity_home = ToTransformMatrix( Vec3d( 0, -0.1, 2 ) );
	auto model = CreateModel( *model_->GetChain(), singularity_home );

	auto singularity_base_group = Model::RevoluteBaseJointGroup( 
		ToTransformMatrix( Vec3d( 0, 0, 2 ) ) );
	auto heuristic = Heuristic::RevoluteBaseHeuristic( model, singularity_base_group );

	VecXd seed(3);
	seed << M_PI / 2, 0, 0;

	VecXd joints( 3 );
	joints << M_PI / 4, 0, 0;

	auto problem = CreateProblem( seed, joints );
	auto result = heuristic.Presolve( problem, Solver::IKRunContext() );

	// Check that the solution is valid
	EXPECT_EQ( result.state, Heuristic::IKHeuristicState::PartialSuccess ) 
		<< "IK should partially succeed for singularity";

	EXPECT_EQ( seed[0], result.joints[0] ) 
		<< "Expected joint= " << seed[0] << std::endl
		<< "Result   joint= " << result.joints[0] << std::endl;
}

// ------------------------------------------------------------

}
