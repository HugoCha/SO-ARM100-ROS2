#include "Heuristic/WristHeuristic.hpp"

#include "Global.hpp"

#include "Heuristic/IKHeuristic.hpp"
#include "Heuristic/IKHeuristicState.hpp"
#include "Heuristic/IKPresolution.hpp"
#include "Model/JointGroup.hpp"
#include "Model/WristTopology.hpp"
#include "Solver/IKProblem.hpp"
#include "Utils/KinematicsUtils.hpp"

namespace SOArm100::Kinematics::Heuristic
{

// ------------------------------------------------------------

WristHeuristic::WristHeuristic(
	Model::KinematicModelConstPtr model,
	Model::JointGroup wrist_group ) :
	IKHeuristic( model ),
	wrist_group_( wrist_group ),
	wristless_group_( ComputeWristlessGroup( model, wrist_group ) )
{
}

// ------------------------------------------------------------

Model::WristTopology WristHeuristic::GetWristTopology() const
{
	if ( wrist_group_.indices.size() > 3 )
		return Model::WristTopology::None;
	return static_cast< Model::WristTopology >( wrist_group_.indices.size() );
}

// ------------------------------------------------------------

IKPresolution WristHeuristic::Presolve(
	const Solver::IKProblem& problem,
	const Solver::IKRunContext& context ) const
{
	Mat4d wrist_center = ComputeWristCenter( problem.seed,  problem.target );
	Mat4d wrist_target = Inverse( wrist_center ) * problem.target;
	Mat3d R_wrist_target = Rotation( wrist_target );

	switch ( GetWristTopology() )
	{
	case Model::WristTopology::Revolute1:
		return SolveRevolute1( problem.seed, R_wrist_target );
	case Model::WristTopology::Revolute2:
		return SolveRevolute2( problem.seed, R_wrist_target );
	case Model::WristTopology::Revolute3:
		return SolveRevolute3( problem.seed, R_wrist_target );
	default:
		break;
	}

	return { problem.seed, IKHeuristicState::PartialSuccess };
}

// ------------------------------------------------------------

Model::JointGroup WristHeuristic::ComputeWristlessGroup(
	Model::KinematicModelConstPtr model,
	const Model::JointGroup wrist_group )
{
	int wristless_count;
	Mat4d wristless_tip;

	if ( wrist_group.indices.empty() )
	{
		wristless_count = model->GetChain()->GetActiveJointCount();
		wristless_tip   = model->GetHomeConfiguration();
	}
	else
	{
		wristless_count = wrist_group.indices[0];
		wristless_tip   = model->GetHomeConfiguration() * Inverse( wrist_group.tip_home );
	}

	std::vector< int > wristless_indices( wristless_count );
	for ( int i = 0; i < wristless_count; i++ )
		wristless_indices[i] = i;

	return { "wristless", wristless_indices, wristless_tip };
}

// ------------------------------------------------------------

Mat4d WristHeuristic::ComputeWristCenter(
	const VecXd& seed,
	const Mat4d& target ) const
{
	Mat4d wrist_center;
	VecXd wristless_joints = wristless_group_.GetGroupJoints( seed );

	model_->GetChain()->ComputeFK(
		wristless_joints,
		wristless_group_.tip_home,
		wrist_center );

	return wrist_center;
}

// ------------------------------------------------------------

IKPresolution WristHeuristic::SolveRevolute1( const VecXd& seed, const Mat3d& R_target ) const
{
	IKPresolution presolution { seed, IKHeuristicState::Fail };
	VecXd wrist_solution = wrist_group_.GetGroupJoints( seed );

	const Vec3d& axis = GetActiveJoint( wrist_group_.indices[0] )->Axis();
	Eigen::AngleAxisd aa( R_target );
	double proj = aa.axis().dot( axis );

	// aa.axis() // axis <=> | proj | = 1
	if ( std::abs( proj ) < 1 - epsilon )
	{
		presolution.state = IKHeuristicState::PartialSuccess;
	}
	else
	{
		wrist_solution << proj * aa.angle();
		presolution.state = IKHeuristicState::Success;
	}

	wrist_group_.SetGroupJoints( wrist_solution, presolution.joints );
	return presolution;
}

// ------------------------------------------------------------

IKPresolution WristHeuristic::SolveRevolute2( const VecXd& seed, const Mat3d& R_target ) const
{
	IKPresolution presolution { seed, IKHeuristicState::Fail };
	VecXd wrist_solution = wrist_group_.GetGroupJoints( seed );

	const Vec3d& e1 = GetActiveJoint( wrist_group_.indices[0] )->Axis();
	const Vec3d& e2 = GetActiveJoint( wrist_group_.indices[1] )->Axis();

	Vec3d v  = e2;
	Vec3d vp = R_target * e2;

	Vec3d v_perp  = v  - e1.dot( v )  * e1;
	Vec3d vp_perp = vp - e1.dot( vp ) * e1;

	if ( v_perp.norm() < epsilon || vp_perp.norm() < epsilon )
	{
		presolution.state = IKHeuristicState::PartialSuccess;
		return presolution;
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
		wrist_solution << q1;
		presolution.state = IKHeuristicState::PartialSuccess;
	}
	else
	{
		wrist_solution << q1, q2;
		presolution.state = IKHeuristicState::Success;
	}

	wrist_group_.SetGroupJoints( wrist_solution, presolution.joints );
	return presolution;
}

// ------------------------------------------------------------

IKPresolution WristHeuristic::SolveRevolute3( const VecXd& seed, const Mat3d& R_target ) const
{
	IKPresolution presolution { seed, IKHeuristicState::Fail };
	VecXd wrist_solution = wrist_group_.GetGroupJoints( seed );

	const Vec3d& e1 = GetActiveJoint( wrist_group_.indices[0] )->Axis();
	const Vec3d& e2 = GetActiveJoint( wrist_group_.indices[1] )->Axis();
	const Vec3d& e3 = GetActiveJoint( wrist_group_.indices[2] )->Axis();

	Mat3d R = R_target;

	// STEP 1: Solve q1 by aligning e3
	Vec3d v  = e3;
	Vec3d vp = R * e3;

	Vec3d v_perp  = v  - e1.dot( v )  * e1;
	Vec3d vp_perp = vp - e1.dot( vp ) * e1;

	if ( v_perp.norm() < epsilon || vp_perp.norm() < epsilon )
	{
		presolution.state = IKHeuristicState::PartialSuccess;
		return presolution;
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
		wrist_solution << q1;
		presolution.state = IKHeuristicState::PartialSuccess;
		wrist_group_.SetGroupJoints( wrist_solution, presolution.joints );
		return presolution;
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
		wrist_solution << q1, q2;
		presolution.state = IKHeuristicState::PartialSuccess;
	}
	else
	{
		wrist_solution << q1, q2, q3;
		presolution.state = IKHeuristicState::Success;
	}

	wrist_group_.SetGroupJoints( wrist_solution, presolution.joints );
	return presolution;
}

// ------------------------------------------------------------

}