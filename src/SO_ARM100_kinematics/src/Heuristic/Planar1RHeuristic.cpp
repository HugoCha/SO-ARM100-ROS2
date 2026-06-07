#include "Heuristic/Planar1RHeuristic.hpp"

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

Planar1RHeuristic::Planar1RHeuristic(
	Model::KinematicModelConstPtr model,
	Model::JointGroup planar_group ) :
	Model::IKJointGroupModelBase( model, planar_group ),
	reference_direction_( ComputeReferenceDirection( model, planar_group ) )
{
}

// ------------------------------------------------------------

Vec3d Planar1RHeuristic::ComputeReferenceDirection(
	Model::KinematicModelConstPtr model,
	const Model::JointGroup planar_group )
{
	const auto& chain = model->GetChain();
	const int n_joints = chain->GetActiveJointCount();

	Vec3d p1 = chain->GetActiveJoint( planar_group.FirstIndex() )->Origin();
	Vec3d p2 = Translation( planar_group.tip_home );

	return ( p2 - p1 ).normalized();
}

// ------------------------------------------------------------

Model::JointConstPtr Planar1RHeuristic::GetJoint() const
{
	return GetChain()->GetActiveJoint( GetGroup().FirstIndex() );
}

// ------------------------------------------------------------

double Planar1RHeuristic::L() const
{
	const auto& link = GetJoint()->GetLink();
	return link.GetLength();
}

// ------------------------------------------------------------

IKPresolution Planar1RHeuristic::Presolve(
	const Solver::IKProblem& problem,
	const Solver::IKRunContext& context ) const
{
	VecXd seed = problem.seed;
	VecXd planar_solution( 1 );

	auto T_group_target = ComputeGroupLocalTarget( seed, problem.target );
	Vec3d p_group_target = Translation( T_group_target );
	double D  = p_group_target.norm();

	// Unreachable
	if ( D > L() )
		return { seed, IKHeuristicState::Fail }
	;

	double value = SignedAngle(
		reference_direction_,
		p_group_target,
		GetJoint()->Axis() );

	if ( !GetJoint()->GetLimits().Within( value ) )
	{
		planar_solution[0] = GetJoint()->GetLimits().Clamp( value );
		GetGroup().SetGroupJoints( planar_solution, seed );
		return { seed, IKHeuristicState::PartialSuccess };
	}

	planar_solution[0] = value;
	GetGroup().SetGroupJoints( planar_solution, seed );
	return { seed, IKHeuristicState::Success };
}

// ------------------------------------------------------------

}