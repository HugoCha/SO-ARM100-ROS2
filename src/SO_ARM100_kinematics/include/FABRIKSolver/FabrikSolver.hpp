#pragma once

#include "Global.hpp"

#include "Model/Pose.hpp"
#include "KinematicsSolver.hpp"

#include <span>
#include <vector>

namespace SOArm100::Kinematics 
{
struct NumericSolverResult;

class FABRIKKinematicsSolver : public KinematicsSolver
{
public:
struct SolverParameters
{
    int max_iterations{20};
    double error_tolerance {5e-3};
};

explicit FABRIKKinematicsSolver() : FABRIKKinematicsSolver( SolverParameters() ) {}
explicit FABRIKKinematicsSolver( SolverParameters parameters );
~FABRIKKinematicsSolver() = default;

FABRIKKinematicsSolver( const FABRIKKinematicsSolver& ) = delete;
FABRIKKinematicsSolver& operator = ( const FABRIKKinematicsSolver& ) = delete;

[[nodiscard]] SolverParameters GetParameters() const {
    return parameters_;
}

void SetParameters( const SolverParameters& parameters ) {
    parameters_ = parameters;
}

[[nodiscard]] NumericSolverResult InverseKinematic(
    const Mat4d& target,
    const std::span< const double >& seed_joints ) const;

protected:
virtual bool InverseKinematicImpl(
	const Mat4d& target,
	const std::span< const double >& seed_joints,
	double* joints ) const override;

private:
struct SolverBuffers
{
    int fabrik_start_idx {0};
    std::vector< double > bone_lengths;
    std::vector< Pose > old_poses;
    std::vector< Pose > poses;
    VecXd joints;
    Mat4d fk;

    explicit SolverBuffers( int n_joints )
    {
        joints.resize( n_joints );
        old_poses.resize( n_joints + 1);
        poses.resize( n_joints + 1 );
        bone_lengths.resize( n_joints );
    }

    int GetSize() const {
        return joints.size();
    }
};

SolverParameters parameters_;
mutable SolverBuffers buffers_;

void PreSolveAzimuthJoints(
    const Vec3d& p_target,
    int& fabrik_start_idx,
    VecXd& joints
) const;

void ComputePoses( 
    const VecXd& joints, 
    std::vector< Pose >& poses ) const;
    
void ComputeBoneLengths( 
    const std::span< const Pose >& poses,
    const std::span< double >& bone_lengths ) const;

void ForwardPass(
    const Vec3d& target,
    const std::span< const double >& bone_lengths,
    int start_idx,
    const std::span< Pose >& poses ) const;

void BackwardPass(
    const Vec3d& base,
    const std::span< const double >& bone_lengths,
    int start_idx,
    const std::span< Pose >& poses ) const;

double RevolutePositionToAngle(
    const Pose& parent_pose,
    const Pose& old_child_pose,
    const Pose& new_child_pose,
    double current_angle ) const;

void ApplyRevoluteJointLimit(
    const Pose&  parent_pose,
    const Limits& limits,
    double& current_angle,
    const std::span< Pose >& child_poses ) const;

double PrismaticPositionToDisplacement(
    const Pose& old_pose,
    const Pose& new_pose,
    double current_displacement ) const;

void UpdateJointValues( 
    const std::span< const Pose >& old_poses,
    int start_idx,
    const std::span< Pose >& poses,
    VecXd& joints ) const;

void PrintBoneLengths( const std::span< const double > bone_lengths ) const;
void PrintPoses( const std::span< const Pose >& poses ) const;
};
}