#include "HybridSolver/HybridSolver.hpp"

#include "RobotModelTestData.hpp"

#include "Global.hpp"
#include "Solver/IKRunContext.hpp"
#include "KinematicTestBase.hpp"
#include "Utils/KinematicsUtils.hpp"

#include <Eigen/src/Core/Matrix.h>
#include <gtest/gtest.h>
#include <memory>
#include <vector>

namespace SOArm100::Kinematics::Test
{

// ------------------------------------------------------------
// ------------------------------------------------------------

class HybridSolverTest : public KinematicTestBase
{
protected:
void SetUp() override
{
	model_ = Data::GetRevolute_Planar2R_Wrist2R_5DOFsRobot();

	// Create a solver
	solver_ = std::make_unique< Solver::HybridSolver >( model_ );
}

void TearDown() override
{
}

std::unique_ptr< Solver::HybridSolver > solver_;
};

// ------------------------------------------------------------
// ------------------------------------------------------------

TEST_F( HybridSolverTest, InverseKinematic_Success )
{
	// Target joints
	VecXd joints( 5 );
	joints << M_PI, M_PI / 2, M_PI / 3, M_PI / 4, M_PI / 5, M_PI / 6;

	// Seed joints
	VecXd seed( 5 );
	seed << 0, 0, 0, 0, 0, 0;

	auto problem = CreateProblem( seed, joints );
	auto result = solver_->Solve( problem, Solver::IKRunContext() );

	auto result_pose = ComputeFK( result.joints );

	// Check that the solution is valid
	EXPECT_TRUE( result.Success() ) << "IK should succeed for reachable target";
	EXPECT_EQ( result.joints.size(), 6 ) << "Result should contain values for all joints";
	EXPECT_TRUE( IsApprox( problem.target, result_pose ) )
	    << "Target=\n" << problem.target << std::endl
	    << "Result=\n" << result_pose << std::endl
	    << "Result joints= " << result.joints.transpose() << std::endl;
}

// ------------------------------------------------------------
// ------------------------------------------------------------

TEST_F( HybridSolverTest, InverseKinematic_Unreachable )
{
	// Create an unreachable target pose
	Mat4d target_pose = Mat4d::Identity();
	target_pose.block< 3, 1 >( 0, 3 ) = Vec3d( 100.0, 0.0, 0.0 );  // Very far away

	// Seed joints
	VecXd seed( 5 );
	seed << 0, 0, 0, 0, 0, 0;

	auto problem = CreateProblem( seed, target_pose );
	auto result = solver_->Solve( problem, Solver::IKRunContext() );

	// Check that the solution is not successful
	EXPECT_FALSE( result.Success() ) << "IK should fail for unreachable target";
}

// ------------------------------------------------------------
// ------------------------------------------------------------

TEST_F( HybridSolverTest, InverseKinematic_WithDifferentSeedJoints )
{
	// Target joints
	VecXd joints( 5 );
	joints << M_PI / 2, M_PI / 2, M_PI / 3, M_PI / 4, M_PI / 5, M_PI / 6;
	auto target_pose = ComputeFK( joints );

	// Test with different seed joints
	std::vector< Eigen::Vector< double, 5 >> seed_joints_list = {
		{ 0, 0, 0, 0, 0 },
		{ M_PI / 4, M_PI / 4, M_PI / 4, M_PI / 4, M_PI / 4 },
		{ -M_PI / 4, -M_PI / 4, -M_PI / 4, -M_PI / 4, -M_PI / 4 },
	};

	for ( const auto& seed_joints : seed_joints_list )
	{
		auto problem = CreateProblem( seed_joints, target_pose );
		auto result = solver_->Solve( problem, Solver::IKRunContext() );

		// Check that the solution is valid
		EXPECT_TRUE( result.Success() );
	}
}

// ------------------------------------------------------------

TEST_F( HybridSolverTest, InverseKinematic_Singularity )
{
	// Create a target pose that might cause a singularity
	Mat4d target_pose = Mat4d::Identity();
	target_pose.block< 3, 3 >( 0, 0 ) = AngleAxis( M_PI / 2, Vec3d( 0, 1, 0 ) ).toRotationMatrix() *
	                                    AngleAxis( M_PI / 2, Vec3d( 1, 0, 0 ) ).toRotationMatrix();
	target_pose.block< 3, 1 >( 0, 3 ) = Vec3d( 0.5, 0.5, 1.0 );

	// Seed joints
	VecXd seed( 5 );
	seed << 0, 0, 0, 0, 0, 0;

	auto problem = CreateProblem( seed, target_pose );
	auto result = solver_->Solve( problem, Solver::IKRunContext() );

	// Check that the solution is valid (either success or singularity)
	EXPECT_TRUE( result.Success() ) << "IK should succeed or detect singularity for target with potential singularity";
}

// ------------------------------------------------------------

} // namespace SOArm100::Kinematics::Test
