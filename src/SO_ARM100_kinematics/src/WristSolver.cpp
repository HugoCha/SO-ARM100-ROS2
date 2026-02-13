#include "WristSolver.hpp"

#include "DLSKinematicsSolver.hpp"
#include "Global.hpp"
#include "KinematicsUtils.hpp"
#include "WristModel.hpp"

#include <memory>
#include <moveit/robot_model/joint_model.hpp>

namespace SOArm100::Kinematics
{

// ------------------------------------------------------------

void WristSolver::Initialize( 
    const WristModel& wrist_model,
    double search_discretization )
{
    assert( wrist_model );

    if ( !p_dls_wrist_solver )
    {
        p_dls_wrist_solver = std::make_unique< DLSKinematicsSolver >();
    }

    p_wrist_model_ = std::make_unique< const WristModel >( wrist_model );

    p_dls_wrist_solver->Initialize( 
        p_wrist_model_->wrist_joints, 
        p_wrist_model_->twists,
        p_wrist_model_->tcp_in_wrist_at_home, 
        search_discretization );
}

// ------------------------------------------------------------

void WristSolver::ComputeWristCenter( const Mat4d& target, Mat4d& wrist_center ) const
{
	wrist_center.noalias() = target * p_wrist_model_->tcp_in_wrist_at_home_inv;
}

// ------------------------------------------------------------

WristSolverResult WristSolver::IK( 
    const Mat4d& target_in_wrist,
    const std::span< const double > seed_joints ) const
{
    assert(p_wrist_model);
    assert(p_dls_wrist_solver);

    const auto& R_target_in_wrist = Rotation( target_in_wrist );

    WristSolverResult result{ p_wrist_model_->count };
	switch ( p_wrist_model_->type )
	{
	case WristType::Revolute1:
        result = SolveRevolute1( *p_wrist_model_, R_target_in_wrist );
        break;
	case WristType::Revolute2:
        result = SolveRevolute2( *p_wrist_model_, R_target_in_wrist );
        break;
	case WristType::Revolute3:
        result = SolveRevolute3( *p_wrist_model_, R_target_in_wrist );
        break;
    default:
        break;
    }

    if ( !result.Success() && !result.Unreachable() )
    {
        result = SolveNumeric(target_in_wrist, seed_joints);
    }

	return result;
}

// ------------------------------------------------------------

WristSolverResult WristSolver::SolveRevolute1( 
    const WristModel& wrist, 
    const Mat3d& R_target_in_wrist ) const
{
	assert( wrist.twists.size() == 1 );
	WristSolverResult result( 1 );

	const Vec3d& axis = wrist.twists[0]->GetAxis();
	Eigen::AngleAxisd aa( R_target_in_wrist );
	double proj = aa.axis().dot( axis );

	// aa.axis() // axis <=> | proj | = 1
	if ( std::abs( proj ) < 1 - epsilon )
	{
		result.state = WristSolverState::Unreachable;
		return result;
	}

	result.joints << proj * aa.angle();
	result.state = WristSolverState::Success;
	return result;
}

// ------------------------------------------------------------

WristSolverResult WristSolver::SolveRevolute2( 
    const WristModel& wrist, 
    const Mat3d& R_target_in_wrist ) const
{
    assert( wrist.twists.size() == 2 );
	WristSolverResult result( 2 );

	const Vec3d& axis1 = wrist.twists[0]->GetAxis();
	const Vec3d& axis2 = wrist.twists[1]->GetAxis();

	const Vec3d& e1 = axis1;
	const Vec3d& e2 = ( axis2 - axis2.dot( e1 ) * e1 ).normalized();
	const Vec3d& e3 = e1.cross( e2 ).normalized();

	Mat3d B;
	B.col( 0 ) = e1;
	B.col( 1 ) = e2;
	B.col( 2 ) = e3;

	Mat3d R = B.transpose() * R_target_in_wrist * B;

	// R = Rz(q1) * Ry(q2)
	double s2 = -R( 2, 0 );
	double c2 =  R( 2, 2 );

	double q2 = atan2( s2, c2 );

	double q1 = atan2( R( 1, 2 ), R( 0, 2 ) );

	const Mat3d& R_check =
		Eigen::AngleAxisd( q1, e1 ).toRotationMatrix() *
		Eigen::AngleAxisd( q2, e2 ).toRotationMatrix();

	if ( ( R_check - R_target_in_wrist ).norm() > epsilon )
	{
		result.state = WristSolverState::Unreachable;
		return result;
	}

	result.joints << q1, q2;
	result.state = WristSolverState::Success;
	return result;
}

// ------------------------------------------------------------

WristSolverResult WristSolver::SolveRevolute3( 
    const WristModel& wrist, 
    const Mat3d& R_target_in_wrist ) const
{
    assert( wrist.twists.size() == 3 );
	WristSolverResult result( 3 );

	Vec3d axis1 = wrist.twists[0]->GetAxis();
	Vec3d axis2 = wrist.twists[1]->GetAxis();
	Vec3d axis3 = wrist.twists[2]->GetAxis();

	Vec3d e1 = axis1;
	Vec3d e2 = ( axis2 - axis2.dot( e1 ) * e1 ).normalized();
	Vec3d e3 = e1.cross( e2 );

	Mat3d B;
	B.col( 0 ) = e1;
	B.col( 1 ) = e2;
	B.col( 2 ) = e3;

	Mat3d R = B.transpose() * R_target_in_wrist * B;

    // R = Rz(q1) * Ry(q2) * Rz(q3)
	double c2 = std::clamp( R( 2, 2 ), -1.0, 1.0 );
	double q2 = acos( c2 );

	if ( std::abs( std::sin( q2 ) ) < epsilon )
	{
		result.state = WristSolverState::Singularity;
		return result;
	}

	double q1 = atan2( R( 1, 2 ), R( 0, 2 ) );
	double q3 = atan2( R( 2, 1 ), -R( 2, 0 ) );

	result.joints << q1, q2, q3;
	result.state = WristSolverState::Success;
	return result;
}

// ------------------------------------------------------------

WristSolverResult WristSolver::SolveNumeric( 
    const Mat4d& target_in_wrist, 
    const std::span< const double > seed_joints ) const
{
    WristSolverResult result { p_wrist_model_->count };

    bool success = p_dls_wrist_solver->InverseKinematic(
        target_in_wrist, 
        seed_joints.subspan( p_wrist_model_->start_index, p_wrist_model_->count ), 
        result.joints );

    result.state = success ? WristSolverState::Success : WristSolverState::Unreachable;
    return result;
}

// ------------------------------------------------------------

}