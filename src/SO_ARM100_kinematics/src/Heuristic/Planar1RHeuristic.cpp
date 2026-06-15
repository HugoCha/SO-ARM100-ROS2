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
	return GetJoint()->GetChildLink()->Length();
}

// ------------------------------------------------------------

IKPresolution Planar1RHeuristic::Presolve(
	const Solver::IKProblem& problem,
	const Solver::IKRunContext& context ) const
{
	IKPresolution presolution;
	presolution.branches = {{ problem.seed }};
	presolution.state = IKHeuristicState::Fail;

	VecXd presolution_joints = problem.seed;
	auto T_group_target = ComputeGroupLocalTarget( problem.seed, problem.target );
	Vec3d p_group_target = Translation( T_group_target );
	double D  = p_group_target.norm();

	// Unreachable
	if ( D > 1.01 * L() )
		return presolution;

	double value = SignedAngle(
		reference_direction_,
		p_group_target,
		GetJoint()->Axis() );

	Vec1d clamp_value( GetJoint()->GetLimits().Clamp( value ) );
	double error = ComputeLocalPositionError( p_group_target, problem.seed, clamp_value );

	if ( !GetJoint()->GetLimits().Within( value ) )
	{
		presolution.state = ( error < 10 * problem.tolerance ) ?
		                    IKHeuristicState::PartialSuccess :
		                    IKHeuristicState::Fail;
	}
	else
	{
		presolution.state = IKHeuristicState::Success;
	}

	GetGroup().SetGroupJoints( clamp_value, presolution_joints );
	presolution.branches = {{ presolution_joints, error }};
	return presolution;
}

// ------------------------------------------------------------

}