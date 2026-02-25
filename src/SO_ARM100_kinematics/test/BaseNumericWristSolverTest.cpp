#include "HybridSolver/BaseNumericWristSolver.hpp"

#include "Global.hpp"
#include "HybridSolver/BaseJointAnalyzer.hpp"
#include "HybridSolver/BaseJointModel.hpp"
#include "HybridSolver/NumericJointsModel.hpp"
#include "HybridSolver/WristModel.hpp"
#include "Joint/JointChain.hpp"
#include "Joint/Twist.hpp"
#include "Joint/Link.hpp"
#include "Joint/Limits.hpp"
#include "SolverResult.hpp"
#include "Utils/Converter.hpp"

#include <gtest/gtest.h>
#include <memory>
#include <ostream>

namespace SOArm100::Kinematics::Test
{

// ------------------------------------------------------------
// Helper function to create a test joint chain
// ------------------------------------------------------------

std::shared_ptr<JointChain> CreateTestJointChain()
{
    // Create a joint chain with 6 joints: 1 base joint + 2 numeric joints + 3 wrist joints
    auto joint_chain = std::make_shared<JointChain>(6);

    // Base joint (revolute around Z-axis)
    Vec3d origin = Vec3d::Zero();
    Mat4d origin_transform = ToTransformMatrix( origin );
    //Vec3d axis = ( origin_transform * Vec3d(0,0,1).homogeneous() ).head( 3 );
    Vec3d axis = Vec3d::UnitZ();
    joint_chain->Add(
        Twist(axis, origin ),
        Link( origin_transform ),
        Limits(-M_PI, M_PI)
    );

    // Numeric joints (2 revolute joints)
    origin = Vec3d( 0,0,0.5);
    origin_transform = ToTransformMatrix( origin );
    //axis = ( origin_transform * Vec3d(0, 1, 0).homogeneous() ).head( 3 );
    axis = Vec3d::UnitY();
    joint_chain->Add(
        Twist(axis, origin ),
        Link( origin_transform ),
        Limits(-M_PI/2, M_PI/2)
    );

    origin = Vec3d( 0,0,1.0);
    origin_transform = ToTransformMatrix( origin );
    //axis = ( origin_transform * Vec3d(0, 1, 0).homogeneous() ).head( 3 );
    axis = Vec3d::UnitY();
    joint_chain->Add(
        Twist(axis, origin ),
        Link( origin_transform ),
        Limits(-M_PI/2, M_PI/2)
    );

    // Wrist joints (3 revolute joints - spherical wrist)
    origin = Vec3d( 0,0,1.5);
    origin_transform = ToTransformMatrix( origin );
    //axis = ( origin_transform * Vec3d(1, 0, 0).homogeneous() ).head( 3 );
    axis = Vec3d::UnitX();
    joint_chain->Add(
        Twist(axis, origin ),
        Link( origin_transform ),
        Limits(-M_PI, M_PI)
    );

    origin = Vec3d( 0,0,1.5);
    origin_transform = ToTransformMatrix( origin );
    //axis = ( origin_transform * Vec3d(0, 1, 0).homogeneous() ).head( 3 );
    axis = Vec3d::UnitY();
    joint_chain->Add(
        Twist(axis, origin ),
        Link( origin_transform ),
        Limits(-M_PI, M_PI)
    );

    origin = Vec3d( 0,0,1.5);
    origin_transform = ToTransformMatrix( origin );
    //axis = ( origin_transform * Vec3d(0, 0, 1).homogeneous() ).head( 3 );
    axis = Vec3d::UnitZ();
    joint_chain->Add(
        Twist(axis, origin ),
        Link( origin_transform ),
        Limits(-M_PI, M_PI)
    );

    return joint_chain;
}

// ------------------------------------------------------------
// ------------------------------------------------------------

class BaseNumericWristSolverTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        // Create a test joint chain
        joint_chain_ = CreateTestJointChain();

        // Create a home configuration
        auto home_configuration = ToTransformMatrix( Vec3d( 0,0,1.5 ) );
        home_configuration_ = std::make_shared<Mat4d>(home_configuration);

        // Create a numeric joints model
        numeric_model_.start_index = 1;       // Two numeric joints
        numeric_model_.count = 2;       // Two numeric joints
        numeric_model_.home_configuration = *home_configuration_;

        // Create a wrist model for the last 3 joints
        wrist_model_.type = WristType::Revolute3;
        wrist_model_.active_joint_start = 3;  // After the base and numeric joints
        wrist_model_.active_joint_count = 3;
        wrist_model_.center_at_home = Vec3d(0, 0, 1.5);
        wrist_model_.tcp_in_wrist_at_home = Mat4d::Identity();
        wrist_model_.tcp_in_wrist_at_home_inv = Mat4d::Identity();

        // Create a base joint model
        base_model_.reference_direction = Vec3d(1.0, 0.0, 0.0);
        // auto base_model = BaseJointAnalyzer::Analyze( *joint_chain_, wrist_model_ ); 
        // base_model_ = *base_model;

        // Initialize the solver
        solver_ = std::make_unique<BaseNumericWristSolver>(
            joint_chain_,
            home_configuration_,
            base_model_,
            numeric_model_,
            wrist_model_
        );
    }

    void TearDown() override
    {
    }

    std::shared_ptr<JointChain> joint_chain_;
    std::shared_ptr<Mat4d> home_configuration_;
    BaseJointModel base_model_;
    NumericJointsModel numeric_model_;
    WristModel wrist_model_;
    std::unique_ptr<BaseNumericWristSolver> solver_;
};

// ------------------------------------------------------------
// ------------------------------------------------------------

TEST_F(BaseNumericWristSolverTest, IK_Success)
{
    // Create a target pose
    Mat4d target_pose;
    VecXd joints(6);
    joints << M_PI / 4, M_PI / 2, M_PI / 2, 0, 0, 0;
    //joints << 0,0,0, 0, 0, 0;
    POE( *joint_chain_, *home_configuration_, joints, target_pose );
    
    // Seed joints
    std::vector<double> seed_joints{0.1,0.1,0.1, 0, 0, 0};
    
    // Solve IK
    SolverResult result = solver_->IK(target_pose, seed_joints, 0.01);
    
    Mat4d result_pose;
    POE( *joint_chain_, *home_configuration_, result.joints, result_pose );
    // Check that the solution is valid
    EXPECT_TRUE(result.Success())
        << "IK should succeed"
        << "Result state= " << ( int )result.state << std::endl
        << "Result joints= " << result.joints.transpose() << std::endl
        << "Target=\n" << target_pose << std::endl
        << "Result=\n" << result_pose << std::endl
        << "Diff=\n" << target_pose - result_pose << std::endl;

    // Check that we have the correct number of joint values
    EXPECT_EQ(result.joints.size(), joint_chain_->GetActiveJointCount())
        << "Solution should contain values for all joints";
}

// ------------------------------------------------------------

TEST_F(BaseNumericWristSolverTest, IK_UnreachableBase)
{
    // Create a target pose that's unreachable for the base joint
    Mat4d target_pose = Mat4d::Identity();
    target_pose.block<3, 1>(0, 3) = Vec3d(100.0, 0.0, 0.0);  // Very far away

    // Seed joints
    std::vector<double> seed_joints{0,0,0, 0, 0, 0};

    // Solve IK
    SolverResult result = solver_->IK(target_pose, seed_joints, 0.01);

    // Check that the solution is unreachable
    EXPECT_TRUE(result.Unreachable()) << "IK should fail for unreachable target";
}

// ------------------------------------------------------------

TEST_F(BaseNumericWristSolverTest, IK_UnreachableNumeric)
{
    // Create a target pose that's unreachable for the numeric joints
    Mat4d target_pose = Mat4d::Identity();
    target_pose.block<3, 3>(0, 0) = Eigen::AngleAxisd(M_PI/4, Vec3d(0, 0, 1)).toRotationMatrix();
    target_pose.block<3, 1>(0, 3) = Vec3d(100.0, 0.0, 1.0);  // Very far away

    // Seed joints
    std::vector<double> seed_joints{0,0,0, 0, 0, 0};

    // Solve IK
    SolverResult result = solver_->IK(target_pose, seed_joints, 0.01);

    // Check that the solution is unreachable
    EXPECT_TRUE(result.Unreachable()) << "IK should fail for unreachable numeric target";
}

// ------------------------------------------------------------

TEST_F(BaseNumericWristSolverTest, IK_UnreachableWrist)
{
    // Create a target pose with an unreachable wrist orientation
    Mat4d target_pose = Mat4d::Identity();
    target_pose.block<3, 3>(0, 0) = Eigen::AngleAxisd(M_PI/4, Vec3d(0, 0, 1)).toRotationMatrix() *
                                    Eigen::AngleAxisd(M_PI/2, Vec3d(1, 0, 0)).toRotationMatrix() *
                                    Eigen::AngleAxisd(M_PI/2, Vec3d(0, 1, 0)).toRotationMatrix();  // Gimbal lock
    target_pose.block<3, 1>(0, 3) = Vec3d(0.5, 0.5, 1.0);

    // Seed joints
    std::vector<double> seed_joints{0,0,0, 0, 0, 0};

    // Solve IK
    SolverResult result = solver_->IK(target_pose, seed_joints, 0.01);

    // Check that the solution is unreachable
    EXPECT_TRUE(result.Unreachable()) << "IK should fail for unreachable wrist orientation";
}

// ------------------------------------------------------------

TEST_F(BaseNumericWristSolverTest, IK_WithDifferentSeedJoints)
{
    // Create a target pose
    Mat4d target_pose = Mat4d::Identity();
    target_pose.block<3, 3>(0, 0) = Eigen::AngleAxisd(M_PI/4, Vec3d(0, 0, 1)).toRotationMatrix();
    target_pose.block<3, 1>(0, 3) = Vec3d(0.5, 0.5, 1.0);

    // Test with different seed joints
    std::vector<std::vector<double>> seed_joints_list = {
        {0,0,0, 0, 0, 0},
        {M_PI/4,M_PI/4,M_PI/4,M_PI/4,M_PI/4,M_PI/4},
        {-M_PI/4,-M_PI/4,-M_PI/4,-M_PI/4,-M_PI/4,-M_PI/4},
    };

    for (const auto& seed_joints : seed_joints_list) {
        // Solve IK
        SolverResult result = solver_->IK(target_pose, seed_joints, 0.01);

        // Check that the solution is valid
        EXPECT_TRUE(result.Success())
            << "IK should succeed target with seed joints: "
            << seed_joints[0];
    }
}

// ------------------------------------------------------------

TEST_F(BaseNumericWristSolverTest, IK_WithDifferentDiscretization)
{
    // Create a target pose
    Mat4d target_pose = Mat4d::Identity();
    target_pose.block<3, 3>(0, 0) = Eigen::AngleAxisd(M_PI/4, Vec3d(0, 0, 1)).toRotationMatrix();
    target_pose.block<3, 1>(0, 3) = Vec3d(0.5, 0.5, 1.0);

    // Seed joints
    std::vector<double> seed_joints{0,0,0, 0, 0, 0};

    // Test with different discretization values
    std::vector<double> discretizations = {0.01, 0.1, 0.5};

    for (const auto& discretization : discretizations) {
        // Solve IK
        SolverResult result = solver_->IK(target_pose, seed_joints, discretization);

        // Check that the solution is valid
        EXPECT_TRUE(result.Success())
            << "IK should either succeed or detect unreachable target with discretization: "
            << discretization;
    }
}

// ------------------------------------------------------------

TEST_F(BaseNumericWristSolverTest, IK_WithNonIdentityHomeConfiguration)
{
    // Create a non-identity home configuration
    Mat4d non_identity_home = Mat4d::Identity();
    non_identity_home.block<3, 3>(0, 0) = Eigen::AngleAxisd(M_PI/4, Vec3d(0, 0, 1)).toRotationMatrix();
    non_identity_home.block<3, 1>(0, 3) = Vec3d(0.1, 0.2, 0.3);

    // Create a solver with the non-identity home configuration
    auto non_identity_solver = std::make_unique<BaseNumericWristSolver>(
        joint_chain_,
        std::make_shared<Mat4d>(non_identity_home),
        base_model_,
        numeric_model_,
        wrist_model_
    );

    // Create a target pose
    Mat4d target_pose = Mat4d::Identity();
    target_pose.block<3, 3>(0, 0) = Eigen::AngleAxisd(M_PI/4, Vec3d(0, 0, 1)).toRotationMatrix();
    target_pose.block<3, 1>(0, 3) = Vec3d(0.5, 0.5, 1.0);

    // Seed joints
    std::vector<double> seed_joints{0,0,0, 0, 0, 0};

    // Solve IK
    SolverResult result = non_identity_solver->IK(target_pose, seed_joints, 0.01);

    // Check that the solution is valid
    EXPECT_TRUE(result.Success())
        << "IK should either succeed or detect unreachable target with non-identity home configuration";
}

// ------------------------------------------------------------

}
