#include "DLSSolver/NumericSolverState.hpp"
#include "Global.hpp"

#include "DLSSolver/NumericSolverResult.hpp"
#include "Joint/Pose.hpp"
#include "Utils/Converter.hpp"
#include "Utils/KinematicsUtils.hpp"
#include "FABRIKSolver/FabrikSolver.hpp"

#include <ios>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <span>
#include <sstream>
#include <string>

namespace SOArm100::Kinematics
{
// ------------------------------------------------------------

static rclcpp::Logger get_logger()
{
	static rclcpp::Logger logger = rclcpp::get_logger( "FabrikKinematicsSolver" );
	return logger;
}

// ------------------------------------------------------------

FABRIKKinematicsSolver::FABRIKKinematicsSolver( SolverParameters parameters ) :
    parameters_( parameters ),
    buffers_( 6 )
{}

// ------------------------------------------------------------

bool FABRIKKinematicsSolver::InverseKinematicImpl(
	const Mat4d& target,
	const std::span< const double >& seed_joints,
	double* joints ) const
{
    auto result = InverseKinematic( target, seed_joints );

	if ( result.Success() )
	{
		std::copy( result.joints.begin(), result.joints.end(), joints );
		return true;
	}

	return false;
}

// ------------------------------------------------------------

NumericSolverResult FABRIKKinematicsSolver::InverseKinematic(
    const Mat4d& target,
    const std::span< const double >& seed_joints ) const 
{
    if ( KinematicsSolver::IsUnreachable( target ) )
	{
		return { NumericSolverState::Failed, {}, -1,  0 };
	}

    const int n_joints = joint_chain_->GetActiveJointCount();

	if ( seed_joints.size() != n_joints )
	{
		RCLCPP_ERROR( get_logger(), "InitializeState: Joint vector size mismatch." );
		return { NumericSolverState::Failed, {}, -1, 0 };;
	}

    if ( buffers_.GetSize() != n_joints )
    {
        buffers_ = SolverBuffers( n_joints );
    }

    const Vec3d p_target = Translation( target );

    buffers_.joints = ToVecXd( seed_joints );
    PreSolveAzimuthJoints( p_target, buffers_.fabrik_start_idx, buffers_.joints );
    ComputePoses( buffers_.joints, buffers_.poses );
    ComputeBoneLengths( buffers_.poses, buffers_.bone_lengths );
    const Vec3d p_base = buffers_.poses[buffers_.fabrik_start_idx].origin;
    
    /*
    std::cout << "Azim joints   = " << buffers_.joints.transpose() << std::endl;
    std::cout << "Base          = " << p_base.transpose() << std::endl;
    std::cout << "Target        = " << p_target.transpose() << std::endl;
    std::cout << "Bones         = ";
    PrintBoneLengths( buffers_.bone_lengths );
    */

    double error;
    for ( int iter = 0; iter < parameters_.max_iterations; iter++ )
    {
        // std::cout << "--------------- Iteration " << iter << " ---------------" << std::endl;
        error = TranslationError( p_target, buffers_.poses[n_joints].origin );
        // std::cout << "Error     = " << error << std::endl;
        // std::cout << "Poses     = " << std::endl;
        // PrintPoses( buffers_.poses );
        if ( error < parameters_.error_tolerance )
            return { NumericSolverState::Converged, buffers_.joints, error, iter };
    
        buffers_.old_poses = buffers_.poses;
        
        ForwardPass( p_target, buffers_.bone_lengths, buffers_.fabrik_start_idx, buffers_.poses );
        // std::cout << "Fwd Poses = " << std::endl;
        // PrintPoses( buffers_.poses );
        BackwardPass( p_base, buffers_.bone_lengths, buffers_.fabrik_start_idx, buffers_.poses );
        // std::cout << "Bwd Poses = " << std::endl;
        // PrintPoses( buffers_.poses );
        UpdateJointValues( buffers_.old_poses, buffers_.fabrik_start_idx, buffers_.poses, buffers_.joints );
        // std::cout << "Joints    = " << buffers_.joints.transpose() << std::endl;
        ComputePoses( buffers_.joints, buffers_.poses );
    }

    return { NumericSolverState::MaxIterations, buffers_.joints, 
             error, parameters_.max_iterations };
}

// ------------------------------------------------------------

void FABRIKKinematicsSolver::PreSolveAzimuthJoints(
    const Vec3d& p_target,
    int& fabrik_start_idx,
    VecXd& joints ) const
{
    const int n = joint_chain_->GetActiveJointCount();
    fabrik_start_idx = 0;

    std::vector< Pose > zero_poses( n + 1 );
    VecXd zero = VecXd::Zero( n );
    joint_chain_->ComputeJointPosesFK( zero, *home_configuration_, zero_poses, buffers_.fk );
    zero_poses[n].origin = Translation( buffers_.fk );

    for ( int i = 0; i < n; i++ )
    {
        const auto& joint = joint_chain_->GetActiveJoint( i );
        if ( !joint->IsRevolute() ) continue;

        Vec3d bone = zero_poses[i+1].origin - zero_poses[i].origin;
        Vec3d axis = zero_poses[i].axis;

        double alignment = std::abs( axis.dot( bone.normalized() ) );
        if ( alignment < 0.99 ) continue;

        if ( fabrik_start_idx == i ) fabrik_start_idx++;

        Vec3d target_dir = p_target - zero_poses[i].origin;
        target_dir -= axis * axis.dot( target_dir );

        if ( target_dir.norm() < error_tolerance ) continue;

        Vec3d ref_dir = bone - axis * axis.dot( bone );
        if ( ref_dir.norm() < 1e-6 )
        {
            Vec3d next_axis = zero_poses[i+1].axis;
            ref_dir = axis.cross( next_axis );
            if ( ref_dir.norm() < 1e-6 ) continue;
        }

        ref_dir.normalize();
        target_dir.normalize();

        double cos_a = std::clamp( ref_dir.dot( target_dir ), -1.0, 1.0 );
        double angle  = std::acos( cos_a );
        double sign   = axis.dot( ref_dir.cross( target_dir ) );
        angle = ( sign >= 0.0 ? angle : -angle );

        const auto& candidates = EvaluateAngleCandidates( *joint, joints[i], angle );
        
        if ( candidates.empty() ) continue;
        joints[i] = candidates[0];
    }
}

// ------------------------------------------------------------

void FABRIKKinematicsSolver::ComputePoses(
    const VecXd& joints,
    std::vector< Pose >& poses ) const
{
    const int n = joints.size();

    joint_chain_->ComputeJointPosesFK( 
        joints, 
        *home_configuration_, 
        poses, 
        buffers_.fk );
    
    poses[ n ].origin.noalias() = Translation( buffers_.fk );
    poses[ n ].axis.setZero();
}

// ------------------------------------------------------------

void FABRIKKinematicsSolver::ComputeBoneLengths( 
    const std::span< const Pose >& poses,
    const std::span< double >& bone_lengths ) const
{
    const int n_joints = poses.size();
    for ( int i = 0; i < n_joints - 1; i++ )
    {
        bone_lengths[i] = ( poses[i+1].origin - poses[i].origin ).norm();
    }
}

// ------------------------------------------------------------

void FABRIKKinematicsSolver::ForwardPass(
    const Vec3d& target,
    const std::span< const double >& bone_lengths,
    int start_idx,
    const std::span< Pose >& poses ) const
{
    const int n = poses.size() - 1;
    poses[ n ].origin = target;

    for ( int i = n - 1; i > start_idx; i-- )
    {
        const auto& joint = joint_chain_->GetActiveJoint( i );
        const Vec3d& axis = poses[i].axis;
        Vec3d dir = poses[i].origin - poses[i+1].origin;

        switch ( joint->GetType() ) 
        {
            case JointType::PRISMATIC:
            {
                double slide = axis.dot( dir );
                slide = joint->GetLimits().Clamp( slide );
                poses[i].origin = poses[i+1].origin + axis * slide;
                break;
            }
            case JointType::REVOLUTE:
            default:
            {
                dir -= axis * axis.dot( dir );
                dir.normalize();
                poses[i].origin = poses[i+1].origin + dir * bone_lengths[i]; 
                break;
            }
        }
    }
}

// ------------------------------------------------------------

void FABRIKKinematicsSolver::BackwardPass(
    const Vec3d& base,
    const std::span< const double >& bone_lengths,
    int start_idx,
    const std::span< Pose >& poses ) const
{
    const int n = poses.size() - 1;
    poses[start_idx].origin = base;

    for ( int i = start_idx + 1; i <= n; i++ )
    {
        const auto& joint = joint_chain_->GetActiveJoint( i - 1 );
        Vec3d dir = poses[i].origin - poses[i-1].origin;

        switch ( joint->GetType() ) 
        {
            case JointType::PRISMATIC:
            {
                const Vec3d& axis = poses[i-1].axis;
                double slide = axis.dot( dir );
                slide = joint->GetLimits().Clamp( slide );
                poses[i].origin = poses[i-1].origin + axis * slide;
                break;
            }
            case JointType::REVOLUTE:
            default:
            {
                poses[i].origin = poses[i-1].origin + dir.normalized() * bone_lengths[i-1]; 
                break;
            }
        }
    }
}

// ------------------------------------------------------------

void FABRIKKinematicsSolver::UpdateJointValues( 
    const std::span< const Pose >& old_poses,
    int start_idx,
    const std::span< Pose >& poses,
    VecXd& joints ) const
{
    const int n = joint_chain_->GetActiveJointCount();

    for ( int i = start_idx; i < n; i++ )
    {
        const auto& joint = joint_chain_->GetActiveJoint( i );
        const auto& limits = joint->GetLimits();
        switch ( joint->GetType() ) 
        {
            case JointType::PRISMATIC:
            {
                double new_displacement = PrismaticPositionToDisplacement(
                    old_poses[i],
                    poses[i],
                    joints[i] );
                joints[i] = limits.Clamp( new_displacement );
                break;
            }
            case JointType::REVOLUTE:
            {
                joints[i] = RevolutePositionToAngle(
                    poses[i], 
                    old_poses[i+1],
                    poses[i+1],
                    joints[i] );
                    
                ApplyRevoluteJointLimit(
                    poses[i], 
                    limits,
                    joints[i],
                    poses.subspan(i + 1, n - (i + 1) ) );
                break;
            }
            default:
                break;
        }
    }
}

// ------------------------------------------------------------

double FABRIKKinematicsSolver::RevolutePositionToAngle(
    const Pose& parent_pose,
    const Pose& old_child_pose,
    const Pose& new_child_pose,
    double current_angle ) const 
{
    Vec3d v_old = old_child_pose.origin - parent_pose.origin;
    Vec3d v_new = new_child_pose.origin - parent_pose.origin;
    
    v_old -= parent_pose.axis * parent_pose.axis.dot( v_old );
    v_new -= parent_pose.axis * parent_pose.axis.dot( v_new );

    double len_old = v_old.norm();
    double len_new = v_new.norm();

    if ( len_new < 1e-9 || len_old < 1e-9 )
        return current_angle;

    v_old /= len_old;
    v_new /= len_new;

    double cos_angle = std::clamp( v_old.dot( v_new ), -1.0, 1.0 );
    double angle = std::acos( cos_angle );
    double sign = parent_pose.axis.dot( v_old.cross( v_new ) );

    return current_angle + ( sign >= 0 ? angle : -angle );
}

// ------------------------------------------------------------

double FABRIKKinematicsSolver::PrismaticPositionToDisplacement(
    const Pose& old_joint_pose,
    const Pose& new_joint_pose,
    double current_displacement ) const
{
    Vec3d delta = new_joint_pose.origin - old_joint_pose.origin;
    return current_displacement + new_joint_pose.axis.dot( delta );
}

// ------------------------------------------------------------

void FABRIKKinematicsSolver::ApplyRevoluteJointLimit(
    const Pose&  parent_pose,
    const Limits& limits,
    double& current_angle,
    const std::span< Pose >& child_poses ) const
{
    if ( !limits.Within( current_angle ) )
    {
        double clamped = limits.Clamp( current_angle );
        double delta   = clamped - current_angle;

        Eigen::AngleAxisd correction( delta, parent_pose.axis );
        for ( size_t k = 0; k < child_poses.size(); k++ )
        {
            Vec3d bone_vec = child_poses[k].origin - parent_pose.origin;
            child_poses[k].origin = parent_pose.origin + ( correction * bone_vec );
        }

        current_angle = clamped;
    }
}

// ------------------------------------------------------------

void FABRIKKinematicsSolver::PrintBoneLengths( 
    const std::span< const double > bone_lengths ) const
{
    const int n_bones = bone_lengths.size();
    std::stringstream bones_ss;
    for ( int i = 0; i < n_bones; i++ )
    {
        bones_ss << bone_lengths[i];
        if ( i != n_bones - 1 )
            bones_ss << ", ";
    }
    std::cout << bones_ss.str() << std::endl;
}

// ------------------------------------------------------------

void FABRIKKinematicsSolver::PrintPoses( const std::span< const Pose >& poses ) const
{
    const int n_poses = poses.size();
    std::stringstream poses_ss;
    poses_ss << std::fixed << std::setprecision(3) << std::showpos;
    for ( int i = 0; i < n_poses; i++ )
    {
        poses_ss << "{ origin = ";
        poses_ss << std::internal << poses[i].origin.transpose();
        poses_ss << ", axis = ";
        poses_ss << std::right << poses[i].axis.transpose();
        poses_ss << " }" << std::endl;
    }
    std::cout << poses_ss.str();
}

// ------------------------------------------------------------

}
