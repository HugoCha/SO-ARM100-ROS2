#include "HybridSolver/HybridKinematicsSolver.hpp"

#include "Global.hpp"
#include "Joint/JointChain.hpp"
#include "Joint/Twist.hpp"
#include "Joint/Link.hpp"
#include "Joint/Limits.hpp"
#include "Utils/KinematicsUtils.hpp"

#include <gtest/gtest.h>
#include <memory>
#include <vector>

namespace SOArm100::Kinematics::Test
{

// ------------------------------------------------------------
// Helper function to create a test joint chain
// ------------------------------------------------------------

std::shared_ptr<JointChain> CreateTestJointChain()
{
    // Create a joint chain with 6 joints: 1 base joint + 3 numeric joints + 2 wrist joints
    auto joint_chain = std::make_shared<JointChain>(6);

    // Base joint (revolute around Z-axis)
    joint_chain->Add(
        Twist(Vec3d(0, 0, 1), Vec3d(0, 0, 0)),
        Link(Mat4d::Identity()),
        Limits(-M_PI, M_PI)
    );

    // Numeric joints (3 joints)
    joint_chain->Add(
        Twist(Vec3d(0, 1, 0), Vec3d(0, 0, 0.5)),
        Link(Mat4d::Identity()),
        Limits(-M_PI/2, M_PI/2)
    );
    joint_chain->Add(
        Twist(Vec3d(0, 1, 0), Vec3d(0, 0, 1.0)),
        Link(Mat4d::Identity()),
        Limits(-M_PI/2, M_PI/2)
    );
    joint_chain->Add(
        Twist(Vec3d(0, 0, 1), Vec3d(0, 0, 1.5)),
        Link(Mat4d::Identity()),
        Limits(-M_PI, M_PI)
    );

    // Wrist joints (2 joints)
    joint_chain->Add(
        Twist(Vec3d(1, 0, 0), Vec3d(0, 0, 1.5)),
        Link(Mat4d::Identity()),
        Limits(-M_PI, M_PI)
    );
    joint_chain->Add(
        Twist(Vec3d(0, 1, 0), Vec3d(0, 0, 1.5)),
        Link(Mat4d::Identity()),
        Limits(-M_PI, M_PI)
    );

    return joint_chain;
}

// ------------------------------------------------------------
// ------------------------------------------------------------

class HybridKinematicsSolverTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        // Create a test joint chain
        joint_chain_ = CreateTestJointChain();

        // Create a home configuration
        home_configuration_ = std::make_shared<Mat4d>(Mat4d::Identity());

        // Create a solver
        solver_ = std::make_unique<HybridKinematicsSolver>();

        // Initialize the solver with the simplified Initialize method
        solver_->Initialize(joint_chain_, home_configuration_, 0.01);
    }

    void TearDown() override
    {
    }

    std::shared_ptr<JointChain> joint_chain_;
    std::shared_ptr<Mat4d> home_configuration_;
    std::unique_ptr<KinematicsSolver> solver_;
};

// ------------------------------------------------------------
// ------------------------------------------------------------

TEST_F(HybridKinematicsSolverTest, InverseKinematic_Success)
{
    // Create a target pose
    VecXd joints(6);
    joints << M_PI, M_PI / 2, M_PI / 3, M_PI / 4, M_PI / 5, M_PI / 6;
    Mat4d target_pose = Mat4d::Identity();
    POE( *joint_chain_, *home_configuration_, joints, target_pose );
    // Seed joints
    std::vector<double> seed_joints {0,0,0,0,0,0};

    // Result joints
    VecXd result_joints;

    // Solve IK
    bool success = solver_->InverseKinematic(target_pose, seed_joints, result_joints);

    // Check that the solution is valid
    EXPECT_TRUE(success) << "IK should succeed for reachable target";
    EXPECT_EQ(result_joints.size(), 6) << "Result should contain values for all joints";
    Mat4d fk;
    POE( *joint_chain_, *home_configuration_, result_joints, fk );
    EXPECT_TRUE( IsApprox( target_pose, fk ) ) 
        << "Target=\n" << target_pose << std::endl
        << "Result=\n" << fk << std::endl
        << "Result joints= " << result_joints.transpose() << std::endl;
}

// ------------------------------------------------------------
// ------------------------------------------------------------

TEST_F(HybridKinematicsSolverTest, InverseKinematic_Unreachable)
{
    // Create an unreachable target pose
    Mat4d target_pose = Mat4d::Identity();
    target_pose.block<3, 1>(0, 3) = Vec3d(100.0, 0.0, 0.0);  // Very far away

    // Seed joints
    std::vector<double> seed_joints {0,0,0,0,0,0};

    // Result joints
    VecXd result_joints;

    // Solve IK
    bool success = solver_->InverseKinematic(target_pose, seed_joints, result_joints);

    // Check that the solution is not successful
    EXPECT_FALSE(success) << "IK should fail for unreachable target";
}

// ------------------------------------------------------------
// ------------------------------------------------------------

TEST_F(HybridKinematicsSolverTest, InverseKinematic_WithDifferentSeedJoints)
{
    // Create a target pose
    Mat4d target_pose = Mat4d::Identity();
    target_pose.block<3, 3>(0, 0) = Eigen::AngleAxisd(M_PI/4, Vec3d(0, 0, 1)).toRotationMatrix();
    target_pose.block<3, 1>(0, 3) = Vec3d(0.5, 0.5, 1.0);

    // Test with different seed joints
    std::vector<std::vector<double>> seed_joints_list = {
        {0,0,0,0,0,0},
        {M_PI/4,M_PI/4,M_PI/4,M_PI/4,M_PI/4,M_PI/4},
        {-M_PI/4,-M_PI/4,-M_PI/4,-M_PI/4,-M_PI/4,-M_PI/4},
    };

    for (const auto& seed_joints : seed_joints_list) {
        // Result joints
        VecXd result_joints;

        // Solve IK
        bool success = solver_->InverseKinematic(target_pose, seed_joints, result_joints);

        // Check that the solution is valid
        EXPECT_TRUE(success) << "IK should succeed for reachable target with seed joints: "
                              << seed_joints[0];
    }
}

// ------------------------------------------------------------
// ------------------------------------------------------------

TEST_F(HybridKinematicsSolverTest, InverseKinematic_WithDifferentDiscretization)
{
    // Test with different discretization values
    std::vector<double> discretizations = {0.01, 0.1, 0.5};

    for (const auto& discretization : discretizations) {
        // Create a new solver for each discretization
        std::unique_ptr<KinematicsSolver> solver = std::make_unique<HybridKinematicsSolver>();
        solver->Initialize(joint_chain_, home_configuration_, discretization);

        // Create a target pose
        Mat4d target_pose = Mat4d::Identity();
        target_pose.block<3, 3>(0, 0) = Eigen::AngleAxisd(M_PI/4, Vec3d(0, 0, 1)).toRotationMatrix();
        target_pose.block<3, 1>(0, 3) = Vec3d(0.5, 0.5, 1.0);

        // Seed joints
        std::vector<double> seed_joints {0,0,0,0,0,0};

        // Result joints
        VecXd result_joints;

        // Solve IK
        bool success = solver->InverseKinematic(target_pose, seed_joints, result_joints );

        // Check that the solution is valid
        EXPECT_TRUE(success) << "IK should succeed for reachable target with discretization: "
                              << discretization;
    }
}

// ------------------------------------------------------------
// ------------------------------------------------------------

TEST_F(HybridKinematicsSolverTest, InverseKinematic_Singularity)
{
    // Create a target pose that might cause a singularity
    Mat4d target_pose = Mat4d::Identity();
    target_pose.block<3, 3>(0, 0) = Eigen::AngleAxisd(M_PI/2, Vec3d(0, 1, 0)).toRotationMatrix() *
                                    Eigen::AngleAxisd(M_PI/2, Vec3d(1, 0, 0)).toRotationMatrix();
    target_pose.block<3, 1>(0, 3) = Vec3d(0.5, 0.5, 1.0);

    // Seed joints
    std::vector<double> seed_joints {0,0,0,0,0,0};

    // Result joints
    VecXd result_joints;

    // Solve IK
    bool success = solver_->InverseKinematic(target_pose, seed_joints, result_joints);

    // Check that the solution is valid (either success or singularity)
    EXPECT_TRUE(success) << "IK should succeed or detect singularity for target with potential singularity";
}

// ------------------------------------------------------------
// ------------------------------------------------------------

TEST_F(HybridKinematicsSolverTest, InverseKinematic_WithNonIdentityHomeConfiguration)
{
    // Create a non-identity home configuration
    Mat4d non_identity_home = Mat4d::Identity();
    non_identity_home.block<3, 3>(0, 0) = Eigen::AngleAxisd(M_PI/4, Vec3d(0, 0, 1)).toRotationMatrix();
    non_identity_home.block<3, 1>(0, 3) = Vec3d(0.1, 0.2, 0.3);

    // Create a solver with the non-identity home configuration
    std::unique_ptr<KinematicsSolver> solver = std::make_unique<HybridKinematicsSolver>();
    solver->Initialize(joint_chain_, std::make_shared<Mat4d>(non_identity_home), 0.01);

    // Create a target pose
    Mat4d target_pose = Mat4d::Identity();
    target_pose.block<3, 3>(0, 0) = Eigen::AngleAxisd(M_PI/4, Vec3d(0, 0, 1)).toRotationMatrix();
    target_pose.block<3, 1>(0, 3) = Vec3d(0.5, 0.5, 1.0);

    // Seed joints
    std::vector<double> seed_joints {0,0,0,0,0,0};

    // Result joints
    VecXd result_joints;

    // Solve IK
    bool success = solver->InverseKinematic(target_pose, seed_joints, result_joints);

    // Check that the solution is valid
    EXPECT_TRUE(success) << "IK should succeed for reachable target with non-identity home configuration";
}

// ------------------------------------------------------------
// ------------------------------------------------------------

} // namespace SOArm100::Kinematics::Test
