#include "HybridSolver/WristSolver.hpp"

#include "DLSSolver/DLSKinematicsSolver.hpp"
#include "DLSSolver/NumericSolverResult.hpp"
#include "Global.hpp"
#include "HybridSolver/WristModel.hpp"
#include "IKinematicsSolver.hpp"
#include "Joint/JointChain.hpp"
#include "SolverResult.hpp"
#include "Utils/KinematicsUtils.hpp"

#include <cassert>
#include <memory>

namespace SOArm100::Kinematics
{

// ------------------------------------------------------------

WristSolver::WristSolver(
	std::shared_ptr< const JointChain > joint_chain,
	std::shared_ptr< const Mat4d > home_configuration,
	const WristModel& wrist_model )
{
	const auto& active_joints = joint_chain->GetActiveJoints();
	auto wrist_start_joint = active_joints[wrist_model.active_joint_start];
	auto wrist_end_joint = active_joints[wrist_model.active_joint_start + wrist_model.active_joint_count - 1];

	if ( active_joints.front() == wrist_start_joint && active_joints.back() == wrist_end_joint )
	{
		joint_chain_ = joint_chain;
	}
	else
	{
		joint_chain_ = std::make_shared< const JointChain >( joint_chain->SubChain( wrist_start_joint, wrist_end_joint ) );
	}

	home_configuration_ = home_configuration;
	wrist_model_ = std::make_unique< const WristModel >( wrist_model );
	dls_wrist_solver_ = std::make_unique< DLSKinematicsSolver >();
	dls_wrist_solver_->Initialize(
		joint_chain_,
		home_configuration_,
		0.01 );
}

// ------------------------------------------------------------

void WristSolver::ComputeWristCenter( const Mat4d& target, Mat4d& wrist_center ) const
{
	wrist_center.noalias() = target * wrist_model_->tcp_in_wrist_at_home_inv;
}

// ------------------------------------------------------------

SolverResult WristSolver::Heuristic(
	const Mat4d& target_in_wrist,
	const std::span< const double >& seed_joints,
	double search_discreatization ) const
{
	const auto& R_target_in_wrist = Rotation( target_in_wrist );
	return SolveAnalytical( *joint_chain_, R_target_in_wrist, seed_joints );
}

// ------------------------------------------------------------

SolverResult WristSolver::IK(
	const Mat4d& target_in_wrist,
	const std::span< const double >& seed_joints,
	double search_discreatization ) const
{
	assert( joint_chain_ );
	assert( home_configuration_ );
	assert( wrist_model_ );
	assert( dls_wrist_solver_ );

	const auto& R_target_in_wrist = Rotation( target_in_wrist );

	auto result = SolveAnalytical( *joint_chain_, R_target_in_wrist, seed_joints );

	if ( !result.Success() && !result.Unreachable() )
	{
		std::vector< double > new_seed_joints( result.joints.begin(), result.joints.end() );
		result = SolveNumeric( target_in_wrist, new_seed_joints );
	}

	Mat4d fk_result;
	CheckSolverResult(
		*joint_chain_,
		*home_configuration_,
		target_in_wrist,
		fk_result,
		result );

	return result;
}

// ------------------------------------------------------------

SolverResult WristSolver::SolveAnalytical( 
	const JointChain& joint_chain, 
	const Mat3d& R_target_in_wrist,
	const std::span< const double >& seed_joints ) const
{
	switch ( wrist_model_->type )
	{
	case WristType::Revolute1:
		return SolveRevolute1( *joint_chain_, R_target_in_wrist, seed_joints );
	case WristType::Revolute2:
		return SolveRevolute2( *joint_chain_, R_target_in_wrist, seed_joints );
	case WristType::Revolute3:
		return SolveRevolute3( *joint_chain_, R_target_in_wrist, seed_joints );
	default:
		assert("Should not run wrist solver with None type");
	}
	return { wrist_model_->active_joint_count };;
}

// ------------------------------------------------------------

SolverResult WristSolver::SolveRevolute1(
	const JointChain& joint_chain,
	const Mat3d& R_target_in_wrist,
	const std::span< const double >& seed_joints ) const
{
	assert( joint_chain.GetActiveJointCount() == 1 );
	SolverResult result( 1 );

	const Vec3d& axis = joint_chain.GetActiveJointTwist( 0 ).GetAxis();
	Eigen::AngleAxisd aa( R_target_in_wrist );
	double proj = aa.axis().dot( axis );

	// aa.axis() // axis <=> | proj | = 1
	if ( std::abs( proj ) < 1 - epsilon )
	{
		result.joints << seed_joints[0];
		result.state = SolverState::Unreachable;
		return result;
	}

	result.joints << proj * aa.angle();
	result.state = SolverState::Success;
	return result;
}

// ------------------------------------------------------------

SolverResult WristSolver::SolveRevolute2(
	const JointChain& joint_chain,
	const Mat3d& R_target,
	const std::span< const double >& seed_joints ) const
{
	assert( joint_chain.GetActiveJointCount() == 2 );
	SolverResult result( 2 );

	const Vec3d& e1 = joint_chain.GetActiveJointTwist( 0 ).GetAxis().normalized();
	const Vec3d& e2 = joint_chain.GetActiveJointTwist( 1 ).GetAxis().normalized();

	Vec3d v  = e2;
	Vec3d vp = R_target * e2;

	Vec3d v_perp  = v  - e1.dot( v )  * e1;
	Vec3d vp_perp = vp - e1.dot( vp ) * e1;

	if ( v_perp.norm() < epsilon || vp_perp.norm() < epsilon )
	{
		result.joints << seed_joints[0], seed_joints[1];
		result.state = SolverState::Singularity;
		return result;
	}

	double q1 = atan2(
		e1.dot( v_perp.cross( vp_perp ) ),
		v_perp.dot( vp_perp )
		);

	Mat3d R1 = Eigen::AngleAxisd( q1, e1 ).toRotationMatrix();
	Mat3d R2 = R1.transpose() * R_target;

	// R2 = I + sin(q2) e2x + ( 1 - cos(q2) ) * ( e2x * e2x )
	// Transpose( e2x ) = -e2x
	// R2 - Transpose( R2 ) = 2 * sin( q2 )
	// trace( R ) = 1 + 2 * cos( q2 )
	Vec3d w = 0.5 * Vec3d(
		R2( 2, 1 ) - R2( 1, 2 ),
		R2( 0, 2 ) - R2( 2, 0 ),
		R2( 1, 0 ) - R2( 0, 1 )
		);

	double q2 = atan2( e2.dot( w ), ( R2.trace() - 1.0 ) * 0.5 );

	Mat3d R_check =
		Eigen::AngleAxisd( q1, e1 ).toRotationMatrix() *
		Eigen::AngleAxisd( q2, e2 ).toRotationMatrix();

	if ( !R_check.isApprox( R_target, rotation_tolerance ) )
	{
		result.joints << q1, seed_joints[1];
		result.state = SolverState::Unreachable;
		return result;
	}

	result.joints << q1, q2;
	result.state = SolverState::Success;

	return result;
}

// ------------------------------------------------------------

SolverResult WristSolver::SolveRevolute3(
	const JointChain& joint_chain,
	const Mat3d& R_target,
	const std::span< const double >& seed_joints ) const
{
	assert( joint_chain.GetActiveJointCount() == 3 );

	SolverResult result( 3 );

	Vec3d e1 = joint_chain.GetActiveJointTwist( 0 ).GetAxis().normalized();
	Vec3d e2 = joint_chain.GetActiveJointTwist( 1 ).GetAxis().normalized();
	Vec3d e3 = joint_chain.GetActiveJointTwist( 2 ).GetAxis().normalized();

	Mat3d R = R_target;

	// STEP 1: Solve q1 by aligning e3
	Vec3d v  = e3;
	Vec3d vp = R * e3;

	Vec3d v_perp  = v  - e1.dot( v )  * e1;
	Vec3d vp_perp = vp - e1.dot( vp ) * e1;

	if ( v_perp.norm() < epsilon || vp_perp.norm() < epsilon )
	{
		result.joints << seed_joints[0], seed_joints[1], seed_joints[2];
		result.state = SolverState::Singularity;
		return result;
	}

	double q1 = atan2( e1.dot( v_perp.cross( vp_perp ) ), v_perp.dot( vp_perp ) );

	Mat3d R1 = Eigen::AngleAxisd( q1, e1 ).toRotationMatrix();
	R = R1.transpose() * R;

	// STEP 2: Solve q2 by aligning e3 again
	v  = e3;
	vp = R * e3;

	v_perp  = v  - e2.dot( v )  * e2;
	vp_perp = vp - e2.dot( vp ) * e2;

	if ( v_perp.norm() < epsilon || vp_perp.norm() < epsilon )
	{
		result.joints << q1, seed_joints[1], seed_joints[2];
		result.state = SolverState::Singularity;
		return result;
	}

	double q2 = atan2( e2.dot( v_perp.cross( vp_perp ) ), v_perp.dot( vp_perp ) );

	Mat3d R2 = Eigen::AngleAxisd( q2, e2 ).toRotationMatrix();
	R = R2.transpose() * R;

	// STEP 3: Extract q3 directly
	Vec3d w = 0.5 * Vec3d(
		R( 2, 1 ) - R( 1, 2 ),
		R( 0, 2 ) - R( 2, 0 ),
		R( 1, 0 ) - R( 0, 1 ) );

	double q3 = atan2( e3.dot( w ), ( R.trace() - 1.0 ) * 0.5 );

	Mat3d R_check =
		Eigen::AngleAxisd( q1, e1 ).toRotationMatrix() *
		Eigen::AngleAxisd( q2, e2 ).toRotationMatrix() *
		Eigen::AngleAxisd( q3, e3 ).toRotationMatrix();

	if ( !R_check.isApprox( R_target, rotation_tolerance ) )
	{
		result.joints << q1, q2, seed_joints[2];
		result.state = SolverState::Unreachable;
		return result;
	}

	result.joints << q1, q2, q3;
	result.state = SolverState::Success;
	return result;
}

// ------------------------------------------------------------

SolverResult WristSolver::SolveNumeric(
	const Mat4d& target_in_wrist,
	const std::span< const double > seed_joints ) const
{
	SolverResult result { wrist_model_->active_joint_count };

	return ToSolverResult( dls_wrist_solver_->InverseKinematic(
		target_in_wrist,
		seed_joints.subspan( wrist_model_->active_joint_start, wrist_model_->active_joint_count ) ) );
}

// ------------------------------------------------------------

}