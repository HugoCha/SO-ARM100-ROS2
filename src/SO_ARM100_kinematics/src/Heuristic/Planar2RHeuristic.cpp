#include "Heuristic/Planar2RHeuristic.hpp"

#include "Global.hpp"

#include "Heuristic/IKHeuristicState.hpp"
#include "Heuristic/IKPresolution.hpp"
#include "Heuristic/IKPresolutionBranch.hpp"
#include "Model/IKJointGroupModelBase.hpp"
#include "Model/Joint/JointGroup.hpp"
#include "Solver/IKProblem.hpp"
#include "Solver/IKRunContext.hpp"
#include "Utils/KinematicsUtils.hpp"
#include "Utils/MathUtils.hpp"

namespace SOArm100::Kinematics::Heuristic
{

// ------------------------------------------------------------

Planar2RHeuristic::Planar2RHeuristic(
	Model::KinematicModelConstPtr model,
	Model::JointGroup planar_group ) :
	Model::IKJointGroupModelBase( model, planar_group ),
	reference_direction_( ComputeReferenceDirection( model, planar_group ) ),
	up_direction_( ComputeUpDirection( reference_direction_, model, planar_group ) ),
	elbow_offset_( ComputeElbowHomeOffset( reference_direction_, model, planar_group ) )
{
}

// ------------------------------------------------------------

Vec3d Planar2RHeuristic::ComputeReferenceDirection(
	Model::KinematicModelConstPtr model,
	const Model::JointGroup planar_group )
{
	auto shoulder_joint = model->GetChain()->GetActiveJoint( planar_group.FirstIndex() );
	auto elbow_joint = model->GetChain()->GetActiveJoint( planar_group.Index( 1 ) );

	Vec3d plane_normal = shoulder_joint->Axis();
	Vec3d p_shoulder = shoulder_joint->Origin();
	Vec3d p_elbow    = elbow_joint->Origin();

	return ProjectPointOnPlane( p_elbow - p_shoulder, Vec3d::Zero(), plane_normal ).normalized();
}

// ------------------------------------------------------------

Vec3d Planar2RHeuristic::ComputeUpDirection(
	const Vec3d& reference_direction,
	Model::KinematicModelConstPtr model,
	const Model::JointGroup planar_group )
{
	Vec3d plane_normal = model->GetChain()->GetActiveJoint( planar_group.FirstIndex() )->Axis();
	return plane_normal.cross( reference_direction );
}

// ------------------------------------------------------------

double Planar2RHeuristic::ComputeElbowHomeOffset(
	const Vec3d& reference_direction,
	Model::KinematicModelConstPtr model,
	const Model::JointGroup planar_group )
{
	auto shoulder_joint = model->GetChain()->GetActiveJoint( planar_group.FirstIndex() );
	auto elbow_joint    = model->GetChain()->GetActiveJoint( planar_group.Index( 1 ) );

	Vec3d plane_normal = shoulder_joint->Axis();

	Vec3d p_elbow    = elbow_joint->Origin();
	Vec3d p_tip      = Translation( planar_group.tip_home );

	return SignedAngle( p_tip - p_elbow, reference_direction, plane_normal );
}

// ------------------------------------------------------------

double Planar2RHeuristic::L1() const
{
	auto shoulder_joint = GetChain()->GetActiveJoint( GetGroup().FirstIndex() );
	auto elbow_joint = GetChain()->GetActiveJoint( GetGroup().Index( 1 ) );
	Vec3d plane_normal = shoulder_joint->Axis();

	Vec3d p_shoulder = shoulder_joint->Origin();
	Vec3d p_elbow    = elbow_joint->Origin();

	return ProjectPointOnPlane( p_elbow - p_shoulder, Vec3d::Zero(), plane_normal ).norm();
}

// ------------------------------------------------------------

double Planar2RHeuristic::L2() const
{
	auto elbow_joint = GetChain()->GetActiveJoint( GetGroup().Index( 1 ) );
	Vec3d plane_normal = elbow_joint->Axis();

	Vec3d p_elbow    = elbow_joint->Origin();
	Vec3d p_tip      = Translation( GetGroup().tip_home );

	return ProjectPointOnPlane( p_tip - p_elbow, Vec3d::Zero(), plane_normal ).norm();
}

// ------------------------------------------------------------

IKPresolution Planar2RHeuristic::Presolve(
	const Solver::IKProblem& problem,
	const Solver::IKRunContext& context ) const
{
	IKPresolution presolution = { {}, IKHeuristicState::Fail };

	auto shoulder_joint = GetChain()->GetActiveJoint( GetGroup().FirstIndex() );
	auto T_group_target = ComputeGroupLocalTarget( problem.seed, problem.target );
	Vec3d p_group_target = Translation( T_group_target );

	Vec3d plane_point = shoulder_joint->Origin();
	Vec3d plane_normal = shoulder_joint->Axis();
	Vec3d wrist_dir = ProjectPointOnPlane( p_group_target, Vec3d::Zero(), plane_normal );

	double x_local = wrist_dir.dot( reference_direction_ );
	double y_local = wrist_dir.dot( up_direction_ );
	double D_squared = x_local * x_local + y_local * y_local;
	double D  = sqrt( D_squared );
	double L1 = Planar2RHeuristic::L1();
	double L2 = Planar2RHeuristic::L2();

	// Unreachable
	if ( D > ( L1 + L2 ) * 1.01 || D * 1.01 < std::abs( L1 - L2 ) )
		return presolution;

	VecXd elbow_up_solution = ComputeElbowUpSolution( x_local, y_local, L1, L2 );
	VecXd elbow_down_solution = ComputeElbowDownSolution( x_local, y_local, L1, L2 );
	std::vector< VecXd > valid_solutions;
	if ( !ValidateAndSelectElbowConfiguration( elbow_up_solution, elbow_down_solution, valid_solutions ) )
	{
		return presolution;
	}

	for ( const auto& valid_solution : valid_solutions )
	{
		IKPresolutionBranch branch = {problem.seed};
		GetGroup().SetGroupJoints( valid_solution, branch.joints );
		presolution.branches.emplace_back( branch );
	}
	
	presolution.state = IKHeuristicState::Success;
	return presolution;
}

// ------------------------------------------------------------

VecXd Planar2RHeuristic::ComputeElbowUpSolution( double x, double y, double L1, double L2 ) const
{
	VecXd up_solution( 2 );

	double c2 = ( x * x + y * y - L1 * L1 - L2 * L2 ) / ( 2 * L1 * L2 );
	double s2 = std::sqrt( std::max( 0.0, 1 - c2 * c2 ) );
	double q2  = std::atan2( s2, c2 );

	double gamma = std::atan2( y, x );
	double beta  = std::atan2( L2 * s2, L1 + L2 * c2 );
	double q1 = gamma - beta;

	up_solution[0] = WrapAngle( q1 );
	up_solution[1] = WrapAngle( q2 + elbow_offset_ );

	return up_solution;
}

// ------------------------------------------------------------

VecXd Planar2RHeuristic::ComputeElbowDownSolution( double x, double y, double L1, double L2 ) const
{
	VecXd down_solution( 2 );

	double c2 = ( x * x + y * y - L1 * L1 - L2 * L2 ) / ( 2 * L1 * L2 );
	double s2 = -std::sqrt( std::max( 0.0, 1 - c2 * c2 ) );
	double q2  = std::atan2( s2, c2 );

	double gamma = std::atan2( y, x );
	double beta  = std::atan2( L2 * s2, L1 + L2 * c2 );
	double q1 = gamma - beta;

	down_solution[0] = WrapAngle( q1 );
	down_solution[1] = WrapAngle( q2 + elbow_offset_ );

	return down_solution;
}

// ------------------------------------------------------------

bool Planar2RHeuristic::ValidateAndSelectElbowConfiguration(
	const VecXd& elbow_up,
	const VecXd& elbow_down,
	std::vector< VecXd >& branches ) const
{
	const auto& shoulder_limits =
		GetChain()->GetActiveJointLimits( GetGroup().FirstIndex() );
	const auto& elbow_limits =
		GetChain()->GetActiveJointLimits( GetGroup().Index( 1 ) );

	bool elbow_up_valid = shoulder_limits.Within( elbow_up[0] )
	                      && elbow_limits.Within( elbow_up[1] );
	bool elbow_down_valid = shoulder_limits.Within( elbow_down[0] )
	                        && elbow_limits.Within( elbow_down[1] );

	if ( !elbow_up_valid && !elbow_down_valid )
	{
		branches = {};
		return false;
	}

	if ( elbow_up_valid )
	{
		branches.emplace_back( elbow_up );
	}
	if ( elbow_down_valid )
	{
		branches.emplace_back( elbow_down );
	}

	return true;
}

// ------------------------------------------------------------

}