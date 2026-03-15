#include "DLSSolver/NumericSolverState.hpp"
#include "Global.hpp"

#include "DLSSolver/NumericSolverResult.hpp"
#include "Utils/Converter.hpp"
#include "Utils/KinematicsUtils.hpp"
#include "FABRIKSolver/FabrikSolver.hpp"

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

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

    const Vec3d p_base = joint_chain_->GetActiveJoint( 0 )->Origin();
    const Vec3d p_target = Translation( target );

    buffers_.joints = ToVecXd( seed_joints );
    ComputeBoneLengths( buffers_.bone_lengths );
    
    double error;
    for ( int iter = 0; iter < parameters_.max_iterations; iter++ )
    {
        ComputePoses( buffers_.joints, buffers_.poses );
        buffers_.old_poses = buffers_.poses;

        error = TranslationError( target, buffers_.fk );
        if ( error < parameters_.error_tolerance )
            return { NumericSolverState::Converged, buffers_.joints, error, iter };

        ForwardPass( p_target, buffers_.bone_lengths, buffers_.poses );
        BackwardPass( p_base, buffers_.bone_lengths, buffers_.poses );
        UpdateJointValues( buffers_.old_poses, buffers_.poses, buffers_.joints );
    }

    return { NumericSolverState::MaxIterations, buffers_.joints, 
             error, parameters_.max_iterations };
}

// ------------------------------------------------------------

void FABRIKKinematicsSolver::ComputePoses(
    const VecXd& joints,
    std::vector< Pose >& poses ) const
{
    const int n = joints.size();

    joint_chain_->ComputeIntermediateFK( 
        joints, 
        *home_configuration_, 
        poses, 
        buffers_.fk );
    
    poses[ n ].origin.noalias() = Translation( buffers_.fk );
    poses[ n ].axis.setZero();
}

// ------------------------------------------------------------

void FABRIKKinematicsSolver::ComputeBoneLengths( std::vector< double >& bone_lengths ) const
{
    const int n_joints = joint_chain_->GetActiveJointCount();
    for ( int i = 0; i < n_joints - 1; i++ )
    {
        bone_lengths[i] = joint_chain_->GetActiveJointLink( i + 1 ).GetLength();
    }
    const Vec3d p_tip = Translation( *home_configuration_ );
    const Vec3d p_last_joints = joint_chain_->GetActiveJoint( n_joints - 1 )->Origin();
    bone_lengths[ n_joints - 1 ] = ( p_tip - p_last_joints ).norm(); 
}

// ------------------------------------------------------------

void FABRIKKinematicsSolver::ForwardPass(
    const Vec3d& target,
    const std::span< const double >& bone_lengths,
    std::vector< Pose >& poses ) const
{
    auto active_joints = joint_chain_->GetActiveJoints();

    const int n = poses.size() - 1;
    poses[ n ].origin = target;

    for ( int i = n - 1; i >= 0; i-- )
    {
        const auto& joint = joint_chain_->GetActiveJoint( i );
        Vec3d dir = poses[i].origin - poses[i+1].origin;

        switch ( joint->GetType() ) 
        {
            case JointType::PRISMATIC:
            {
                const Vec3d& axis = poses[i].axis;
                double slide = axis.dot( dir );
                slide = joint->GetLimits().Clamp( slide );
                poses[i].origin = poses[i+1].origin + axis * slide;
                break;
            }
            case JointType::REVOLUTE:
            default:
            {
                poses[i].origin = poses[i+1].origin + dir.normalized() * bone_lengths[i]; 
                break;
            }
        }
    }
}

// ------------------------------------------------------------

void FABRIKKinematicsSolver::BackwardPass(
    const Vec3d& base,
    const std::span< const double >& bone_lengths,
    std::vector< Pose >& poses ) const
{
    auto active_joints = joint_chain_->GetActiveJoints();

    const int n = poses.size() - 1;
    poses[0].origin = base;

    for ( int i = 1; i <= n; i++ )
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
    std::vector< Pose >& poses,
    VecXd& joints ) const
{
    auto active_joints = joint_chain_->GetActiveJoints();
    const int n = active_joints.size();

    for ( int i = 0; i < n; i++ )
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
                double new_angle = RevolutePositionToAngle(
                    poses[i], 
                    old_poses[i+1],
                    poses[i+1],
                    joints[i] );
                joints[i] = limits.Clamp( new_angle );

                ApplyRevoluteJointLimit(
                    poses[i], 
                    limits,
                    joints[i],
                    poses[i + 1] );
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
    double current_angle,
    Pose& child_pose ) const
{
    if ( !limits.Within( current_angle ) )
    {
        double clamped = limits.Clamp( current_angle );
        double delta   = clamped - current_angle;

        Eigen::AngleAxisd correction( delta, parent_pose.axis );
        Vec3d bone = child_pose.origin - parent_pose.origin;
        bone = correction * bone;

        child_pose.origin = parent_pose.origin + bone;
    }
}

// ------------------------------------------------------------

}
