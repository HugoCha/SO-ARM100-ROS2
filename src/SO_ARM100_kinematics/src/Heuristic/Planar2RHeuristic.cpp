#include "Heuristic/Planar2RHeuristic.hpp"

#include "Global.hpp"

#include "Heuristic/IKHeuristicState.hpp"
#include "Heuristic/IKPresolution.hpp"
#include "Model/IKJointGroupModelBase.hpp"
#include "Model/Joint/JointGroup.hpp"
#include "Solver/IKProblem.hpp"
#include "Solver/IKRunContext.hpp"
#include "Utils/KinematicsUtils.hpp"
#include "Utils/MathUtils.hpp"

#include <cmath>

namespace SOArm100::Kinematics::Heuristic
{

// ------------------------------------------------------------

Planar2RHeuristic::Planar2RHeuristic(
	Model::KinematicModelConstPtr model,
	Model::JointGroup planar_group ) :
	Model::IKJointGroupModelBase( model, planar_group ),
	reference_direction_( ComputeReferenceDirection( model, planar_group ) ),
	up_direction_( ComputeUpDirection( reference_direction_, model, planar_group ) ),
	elbow_offset_( ComputeElbowHomeOffset( model, planar_group ) )
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
	Vec3d p_elbow 	 = elbow_joint->Origin();

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
    Model::KinematicModelConstPtr model,
    const Model::JointGroup planar_group )
{
    auto shoulder_joint = model->GetChain()->GetActiveJoint( planar_group.FirstIndex() );
    auto elbow_joint    = model->GetChain()->GetActiveJoint( planar_group.Index( 1 ) );

    Vec3d plane_normal = shoulder_joint->Axis();

    Vec3d p_shoulder = shoulder_joint->Origin();
    Vec3d p_elbow    = elbow_joint->Origin();
    Vec3d p_tip      = Translation( planar_group.tip_home );

    Vec3d v1 = ProjectPointOnPlane( p_elbow - p_shoulder, Vec3d::Zero(), plane_normal ).normalized();
    Vec3d v2 = ProjectPointOnPlane( p_tip   - p_elbow,   Vec3d::Zero(), plane_normal ).normalized();

    double c = v1.dot( v2 );
    double s = plane_normal.dot( v1.cross( v2 ) );
    return std::atan2( s, c );
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
	Vec3d p_tip 	 = Translation( GetGroup().tip_home );

	return ProjectPointOnPlane( p_tip - p_elbow, Vec3d::Zero(), plane_normal ).norm();
}

// ------------------------------------------------------------

IKPresolution Planar2RHeuristic::Presolve(
	const Solver::IKProblem& problem,
	const Solver::IKRunContext& context ) const
{
	VecXd seed = problem.seed;
	VecXd planar_solution( 2 );
	auto shoulder_joint = GetChain()->GetActiveJoint( GetGroup().FirstIndex() );
	auto T_group_target = ComputeGroupLocalTarget( seed, problem.target );
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
	if ( D > L1 + L2 || D < std::abs( L1 - L2 ) )
		return {seed, IKHeuristicState::Fail };

	VecXd elbow_up_solution( 2 );
	VecXd elbow_down_solution( 2 );

	// Elbow
	double cos_beta = ( L1 * L1 + L2 * L2 - D_squared ) / ( 2.0 * L1 * L2 );
	double beta = std::acos( std::clamp( cos_beta, -1.0, 1.0 ) );
	double elbow_up = M_PI - ( beta + elbow_offset_ );
	double elbow_down = -M_PI + beta + elbow_offset_;

	// Shoulder
	double alpha_target  = std::atan2( y_local, x_local );
	double cos_alpha_int = ( L1 * L1 + D_squared - L2 * L2 ) / ( 2.0 * L1 * D );
	double alpha_int = std::acos( std::clamp( cos_alpha_int, -1.0, 1.0 ) );
	double shoulder_elbow_up = alpha_target - alpha_int;
	double shoulder_elbow_down = alpha_target + alpha_int;

	elbow_up_solution[0] = shoulder_elbow_up;
	elbow_up_solution[1] = elbow_up;

	elbow_down_solution[0] = shoulder_elbow_down;
	elbow_down_solution[1] = elbow_down;

	VecXd planar_seed = GetGroup().GetGroupJoints( problem.seed );
	if ( !ValidateAndSelectElbowConfiguration( 
			planar_seed, 
			elbow_up_solution, 
			elbow_down_solution, 
			planar_solution ) )
	{
		GetGroup().SetGroupJoints( planar_solution, seed );
		return { seed, IKHeuristicState::PartialSuccess };
	}

	
	std::cout << "Planar solution = \n" << planar_solution << std::endl;

	GetGroup().SetGroupJoints( planar_solution, seed );
	return { seed, IKHeuristicState::Success };
}

// ------------------------------------------------------------

bool Planar2RHeuristic::ValidateAndSelectElbowConfiguration(
	const VecXd& planar_seed,
	const VecXd& elbow_up,
	const VecXd& elbow_down,
	VecXd& solution ) const
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
		solution = planar_seed;
		return false;
	}
	else if ( !elbow_up_valid )
	{
		solution = elbow_down;
		return true;
	}
	else if ( !elbow_down_valid )
	{
		solution = elbow_up;
		return true;
	}

	solution = ( planar_seed[1] >= 0.0 ) ? elbow_up : elbow_down;
	return true;
}

// ------------------------------------------------------------

}