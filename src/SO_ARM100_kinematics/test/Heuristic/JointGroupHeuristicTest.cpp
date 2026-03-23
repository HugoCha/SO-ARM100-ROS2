#include <gtest/gtest.h>

#include "Global.hpp"
#include "Heuristic/IKPresolution.hpp"
#include "RobotModelTestData.hpp"

#include "Heuristic/JointGroupHeuristic.hpp"
#include "Model/JointGroup.hpp"
#include "Model/KinematicModel.hpp" // Assuming this is where KinematicModelConstPtr is defined
#include "Solver/IKProblem.hpp"
#include "Solver/IKRunContext.hpp"

namespace SOArm100::Kinematics::Test
{

// ------------------------------------------------------------
// ------------------------------------------------------------

class DummyJointGroupHeuristic : public Heuristic::JointGroupHeuristic
{
public:
DummyJointGroupHeuristic( Model::KinematicModelConstPtr model, Model::JointGroup group ) :
	Heuristic::JointGroupHeuristic( model, group )
{
}

virtual Heuristic::IKPresolution Presolve(
	const Solver::IKProblem& problem,
	const Solver::IKRunContext& context ) const override
{
	return Heuristic::IKPresolution{};
}
};

// ------------------------------------------------------------
// ------------------------------------------------------------

class JointGroupHeuristicTest : public ::testing::Test
{
protected:
void SetUp() override {
	model_ = Data::GetRevolute_Planar2R_Wrist3R_6DOFsRobot();
	group_ = Model::PlanarNRJointGroup( 1, 2, Mat4d::Identity() );
}

void TearDown() override {
	// Clean up if needed
}

Model::KinematicModelConstPtr model_;
Model::JointGroup group_;
};

// ------------------------------------------------------------

TEST_F( JointGroupHeuristicTest, Constructor )
{
	DummyJointGroupHeuristic heuristic( model_, group_ );

	// Check if the group is correctly set
	ASSERT_EQ( heuristic.GetGroup().name, "planar" );
	ASSERT_EQ( heuristic.GetGroup().Size(), 2 );
}

// ------------------------------------------------------------

TEST_F( JointGroupHeuristicTest, GetAncestor )
{
	DummyJointGroupHeuristic heuristic( model_, group_ );

	auto ancestor = heuristic.GetAncestor();
	ASSERT_TRUE( ancestor.has_value() );
	ASSERT_EQ( ancestor->name, "ancestor" );
	ASSERT_EQ( ancestor->FirstIndex(), 0 );
	ASSERT_EQ( ancestor->Size(), 1 );
}

// ------------------------------------------------------------

TEST_F( JointGroupHeuristicTest, GetSuccessor )
{
	DummyJointGroupHeuristic heuristic( model_, group_ );

	auto successor = heuristic.GetSuccessor();
	ASSERT_TRUE( successor.has_value() );
	ASSERT_EQ( successor->name, "successor" );
	ASSERT_EQ( successor->FirstIndex(), 3 );
	ASSERT_EQ( successor->LastIndex(), 5 );
	ASSERT_EQ( successor->Size(), 3 );
}

// ------------------------------------------------------------

}
