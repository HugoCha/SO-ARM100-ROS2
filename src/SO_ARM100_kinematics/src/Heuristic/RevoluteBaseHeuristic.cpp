#include "Heuristic/RevoluteBaseHeuristic.hpp"

#include "Global.hpp"
#include "Heuristic/IKHeuristic.hpp"
#include "Heuristic/IKHeuristicState.hpp"
#include "Heuristic/IKPresolution.hpp"
#include "Heuristic/JointGroupHeuristic.hpp"
#include "Model/JointGroup.hpp"
#include "Solver/IKProblem.hpp"
#include "Utils/KinematicsUtils.hpp"

namespace SOArm100::Kinematics::Heuristic
{

// ------------------------------------------------------------

RevoluteBaseHeuristic::RevoluteBaseHeuristic(
	Model::KinematicModelConstPtr model,
	const Model::RevoluteBaseJointGroup& revolute_base_group ) :
	JointGroupHeuristic( model, revolute_base_group )
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

	const auto& base_origin = base_joint->Origin();
	const auto& p_tip_home = Translation( GetGroup().tip_home );
	const Vec3d& r = p_tip_home - base_origin;
	const auto& base_axis = base_joint->Axis();
	Vec3d reference_direction = r - r.dot( base_axis ) * base_axis;

	if ( reference_direction.norm() < epsilon )
	{
		auto next_joint = model_->GetChain()->GetNextJoint( base_joint );

		if ( !next_joint )
			return Vec3d::Zero();

		const auto& next_joint_axis = next_joint->Axis();
		reference_direction = base_axis.cross( next_joint_axis );

		if ( reference_direction.norm() < epsilon )
			return Vec3d::Zero();
	}

	return reference_direction.normalized();
}

// ------------------------------------------------------------

IKPresolution RevoluteBaseHeuristic::Presolve(
	const Solver::IKProblem& problem,
	const Solver::IKRunContext& context ) const
{
	IKPresolution presolution{ problem.seed, IKHeuristicState::PartialSuccess };

	auto base_joint = GetBaseJoint();

	auto p_wrist_center = ComputeGroupTarget( problem.seed, problem.target );

	const Vec3d& omega = base_joint->Axis();
	const Vec3d& r = p_wrist_center - base_joint->Origin();
	const Vec3d& r_proj = ( r - r.dot( omega ) * omega ).normalized();

	if ( r_proj.norm() < epsilon )
	{
		presolution.joints = { problem.seed };
		presolution.state = IKHeuristicState::PartialSuccess;
	}
	else
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