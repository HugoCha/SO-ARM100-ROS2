#include "Heuristic/RevoluteBaseHeuristic.hpp"

#include "Global.hpp"

#include "Heuristic/IKHeuristicState.hpp"
#include "Heuristic/IKPresolution.hpp"
#include "Model/IKJointGroupModelBase.hpp"
#include "Model/JointGroup.hpp"
#include "Solver/IKProblem.hpp"
#include "Utils/KinematicsUtils.hpp"
#include "Utils/MathUtils.hpp"

namespace SOArm100::Kinematics::Heuristic
{

// ------------------------------------------------------------

RevoluteBaseHeuristic::RevoluteBaseHeuristic(
	Model::KinematicModelConstPtr model,
	const Model::JointGroup& revolute_base_group ) :
	Model::IKJointGroupModelBase( model, revolute_base_group )
{
	reference_direction_ = ComputeReferenceDirection();
}

// ------------------------------------------------------------

const Model::Joint* RevoluteBaseHeuristic::GetBaseJoint() const
{
	return model_->GetChain()->GetActiveJoint( GetGroup().FirstIndex() ).get();
}

// ------------------------------------------------------------

Vec3d RevoluteBaseHeuristic::ComputeReferenceDirection() const
{
	auto base_joint = GetBaseJoint();
	if ( !base_joint )
		return Vec3d::Zero();

	Mat4d tip_home = GetGroup().tip_home;
	Vec3d reference_direction = ComputeDirection( tip_home );

	if ( reference_direction.norm() < epsilon )
	{
		return Vec3d::Zero();
	}

	return reference_direction.normalized();
}

// ------------------------------------------------------------

Vec3d RevoluteBaseHeuristic::ComputeDirection( const Mat4d& T_tip ) const
{
	const auto& p_tip = Translation( T_tip );
	const auto& base_origin =  GetBaseJoint()->Origin();
	const auto& base_axis = GetBaseJoint()->Axis();
	Vec3d direction = ProjectPointOnPlane( p_tip, base_origin, base_axis );

	if ( direction.norm() < epsilon )
		return Vec3d::Zero();

	return direction.normalized();
}

// ------------------------------------------------------------

IKPresolution RevoluteBaseHeuristic::Presolve(
	const Solver::IKProblem& problem,
	const Solver::IKRunContext& context ) const
{
	IKPresolution presolution{ problem.seed, IKHeuristicState::PartialSuccess };
	if ( reference_direction_.norm() < epsilon )
		return presolution;

	auto base_joint = GetBaseJoint();

	auto wrist_center = ComputeGroupWorldTarget( problem.seed, problem.target );

	const Vec3d& omega = base_joint->Axis();
	const Vec3d& r_proj = ComputeDirection( wrist_center );

	if ( r_proj.norm() > epsilon )
	{
		double s_theta = omega.dot( reference_direction_.cross( r_proj ) );
		double c_theta = reference_direction_.dot( r_proj );

		double theta = atan2( s_theta, c_theta );
		const auto& candidates = EvaluateAngleCandidates( *base_joint, problem.seed[0], theta );
		if ( !candidates.empty() )
		{
			presolution.joints = problem.seed;
			presolution.joints[0] = candidates[0];
			presolution.state = IKHeuristicState::Success;
		}
		else
		{
			presolution.joints = {};
			presolution.state = IKHeuristicState::Fail;
		}
	}

	return presolution;
}

// ------------------------------------------------------------

}