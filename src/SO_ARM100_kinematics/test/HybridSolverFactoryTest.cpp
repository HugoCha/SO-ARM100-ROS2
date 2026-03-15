#include "HybridSolver/HybridSolverFactory.hpp"

#include "RobotModelTestData.hpp"
#include "Joint/JointChain.hpp"
#include "HybridSolver/BaseJointModel.hpp"
#include "HybridSolver/BaseJointSolver.hpp"
#include "HybridSolver/BaseNumericWristSolver.hpp"
#include "HybridSolver/BaseWristSolver.hpp"
#include "HybridSolver/HybridSolverConfiguration.hpp"
#include "HybridSolver/NumericJointsModel.hpp"
#include "HybridSolver/NumericJointsSolver.hpp"
#include "HybridSolver/NumericWristSolver.hpp"
#include "HybridSolver/WristModel.hpp"
#include "HybridSolver/WristSolver.hpp"

#include <gtest/gtest.h>
#include <memory>

namespace SOArm100::Kinematics::Test
{

// ------------------------------------------------------------
// ------------------------------------------------------------

class HybridSolverFactoryTest : public ::testing::Test
{
protected:
void SetUp() override
{
	// Create a test joint chain
	joint_chain_ = std::make_shared< JointChain >( Data::Create5DofRobotJointChain() );

	// Create a home configuration
	home_configuration_ = std::make_shared< Mat4d >( Mat4d::Identity() );

	// Create a base joint model
	base_model_.reference_direction = Vec3d( 1.0, 0.0, 0.0 );

	// Create a numeric joints model
	numeric_model_.count = 3;           // Three numeric joints
	numeric_model_.home_configuration = *home_configuration_;

	// Create a wrist model for the last 2 joints
	wrist_model_.type = WristType::Revolute2;
	wrist_model_.active_joint_start = 4;      // After the base and numeric joints
	wrist_model_.active_joint_count = 2;
	wrist_model_.center_at_home = Vec3d( 0, 0, 1.5 );
	wrist_model_.tcp_in_wrist_at_home = Mat4d::Identity();
	wrist_model_.tcp_in_wrist_at_home_inv = Mat4d::Identity();
}

void TearDown() override
{
}

std::shared_ptr< JointChain > joint_chain_;
std::shared_ptr< Mat4d > home_configuration_;
BaseJointModel base_model_;
NumericJointsModel numeric_model_;
WristModel wrist_model_;
};

// ------------------------------------------------------------
// ------------------------------------------------------------

TEST_F( HybridSolverFactoryTest, Get_BaseSolver )
{
	// Create a configuration for base solver
	HybridSolverConfiguration config;
	config.solver_flags = HybridSolverFlags::Base;
	config.base_joint_model = base_model_;

	// Create the solver
	HybridSolverFactory factory;
	auto solver = HybridSolverFactory::Get( joint_chain_, home_configuration_, config );

	// Check that the solver is of the correct type
	EXPECT_NE( solver, nullptr ) << "Solver should not be null";
	EXPECT_NE( dynamic_cast< BaseJointSolver* >( solver.get() ), nullptr )
	    << "Solver should be of type BaseJointSolver";
}

// ------------------------------------------------------------

TEST_F( HybridSolverFactoryTest, Get_WristSolver )
{
	// Create a configuration for wrist solver
	HybridSolverConfiguration config;
	config.solver_flags = HybridSolverFlags::Wrist;
	config.wrist_model = wrist_model_;

	// Create the solver
	HybridSolverFactory factory;
	auto solver = factory.Get( joint_chain_, home_configuration_, config );

	// Check that the solver is of the correct type
	EXPECT_NE( solver, nullptr ) << "Solver should not be null";
	EXPECT_NE( dynamic_cast< WristSolver* >( solver.get() ), nullptr )
	    << "Solver should be of type WristSolver";
}

// ------------------------------------------------------------

TEST_F( HybridSolverFactoryTest, Get_BaseWristSolver )
{
	// Create a configuration for base-wrist solver
	HybridSolverConfiguration config;
	config.solver_flags = HybridSolverFlags::BaseWrist;
	config.base_joint_model = base_model_;
	config.wrist_model = wrist_model_;

	// Create the solver
	HybridSolverFactory factory;
	auto solver = factory.Get( joint_chain_, home_configuration_, config );

	// Check that the solver is of the correct type
	EXPECT_NE( solver, nullptr ) << "Solver should not be null";
	EXPECT_NE( dynamic_cast< BaseWristSolver* >( solver.get() ), nullptr )
	    << "Solver should be of type BaseWristSolver";
}

// ------------------------------------------------------------

TEST_F( HybridSolverFactoryTest, Get_BaseNumericWristSolver )
{
	// Create a configuration for base-numeric-wrist solver
	HybridSolverConfiguration config;
	config.solver_flags = HybridSolverFlags::BaseNumericWrist;
	config.base_joint_model = base_model_;
	config.numeric_joints_model = numeric_model_;
	config.wrist_model = wrist_model_;

	// Create the solver
	HybridSolverFactory factory;
	auto solver = factory.Get( joint_chain_, home_configuration_, config );

	// Check that the solver is of the correct type
	EXPECT_NE( solver, nullptr ) << "Solver should not be null";
	EXPECT_NE( dynamic_cast< BaseNumericWristSolver* >( solver.get() ), nullptr )
	    << "Solver should be of type BaseNumericWristSolver";
}

// ------------------------------------------------------------

TEST_F( HybridSolverFactoryTest, Get_NumericSolver )
{
	// Create a configuration for numeric solver
	HybridSolverConfiguration config;
	config.solver_flags = HybridSolverFlags::Numeric;
	config.numeric_joints_model = numeric_model_;

	// Create the solver
	HybridSolverFactory factory;
	auto solver = factory.Get( joint_chain_, home_configuration_, config );

	// Check that the solver is of the correct type
	EXPECT_NE( solver, nullptr ) << "Solver should not be null";
	EXPECT_NE( dynamic_cast< NumericJointsSolver* >( solver.get() ), nullptr )
	    << "Solver should be of type NumericJointsSolver";
}

// ------------------------------------------------------------

TEST_F( HybridSolverFactoryTest, Get_DefaultSolver )
{
	// Create a configuration with no specific flag
	HybridSolverConfiguration config;
	config.solver_flags = static_cast< HybridSolverFlags >( -1 );  // Invalid flag
	config.numeric_joints_model = numeric_model_;  // Default to numeric solver

	// Create the solver
	HybridSolverFactory factory;
	auto solver = factory.Get( joint_chain_, home_configuration_, config );

	// Check that the solver is of the correct type (should default to numeric)
	EXPECT_NE( solver, nullptr ) << "Solver should not be null";
	EXPECT_NE( dynamic_cast< NumericJointsSolver* >( solver.get() ), nullptr )
	    << "Solver should default to type NumericJointsSolver";
}

// ------------------------------------------------------------

} // namespace SOArm100::Kinematics::Test
