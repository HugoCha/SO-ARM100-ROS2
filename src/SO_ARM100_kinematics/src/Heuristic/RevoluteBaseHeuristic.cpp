#include "Heuristic/RevoluteBaseHeuristic.hpp"

#include "Global.hpp"

#include "Heuristic/IKHeuristicState.hpp"
#include "Heuristic/IKPresolution.hpp"
#include "Model/IKJointGroupModelBase.hpp"
#include "Model/Joint/JointGroup.hpp"
#include "Solver/IKProblem.hpp"
#include "Utils/KinematicsUtils.hpp"
#include "Utils/MathUtils.hpp"
#include "Utils/StringConverter.hpp"
#include <algorithm>

namespace SOArm100::Kinematics::Heuristic
{

// ------------------------------------------------------------

RevoluteBaseHeuristic::RevoluteBaseHeuristic(
	Model::KinematicModelConstPtr model,
	const Model::JointGroup& revolute_base_group ) :
	Model::IKJointGroupModelBase( model, revolute_base_group )
{
	auto base_joint = GetBaseJoint();
	auto shoulder_joint = GetShoulderJoint();
	
	if ( !base_joint || !shoulder_joint )
	{
		reference_direction_ = Vec3d::Zero();
		return;
	}
	
	const auto& base_link = base_joint->GetLink();
	
	Vec3d omega_b = base_joint->Axis();
	Vec3d omega_s = shoulder_joint->Axis();
	Vec3d O_b = base_joint->Origin();
	Vec3d O_s = shoulder_joint->Origin();

	Vec3d wc0 = Translation( revolute_base_group.tip_home );

	shoulder_offset_ = omega_s.dot( wc0 );
	reference_direction_ = omega_b.cross( omega_s );
}

// ------------------------------------------------------------

const Model::Joint* RevoluteBaseHeuristic::GetBaseJoint() const
{
	return GetChain()->GetActiveJoint( GetGroup().FirstIndex() ).get();
}

// ------------------------------------------------------------

const Model::Joint* RevoluteBaseHeuristic::GetShoulderJoint() const
{
	return GetChain()->GetActiveJoint( GetGroup().FirstIndex() + 1 ).get();
}

// ------------------------------------------------------------

Vec3d RevoluteBaseHeuristic::ComputeDirection( const Mat4d& T_tip ) const
{
	const auto& p_tip = Translation( T_tip );
	const auto& base_origin =  GetBaseJoint()->Origin();
	const auto& base_axis = GetBaseJoint()->Axis();
	Vec3d direction = ProjectPointOnPlane( p_tip, base_origin, base_axis ) - base_origin;

	if ( direction.norm() < epsilon )
		return Vec3d::Zero();

	return direction;
}

// ------------------------------------------------------------

double RevoluteBaseHeuristic::ComputeAlpha(  
	const Vec3d& axis,
	const Vec3d& ref_direction, 
	const Vec3d& r_proj )
{
	return SignedAngle( ref_direction, r_proj, axis );
}

// ------------------------------------------------------------

double RevoluteBaseHeuristic::ComputeBeta( 
	double shoulder_offset, 
	const Vec3d& r_proj )
{
	return ( r_proj.norm() > epsilon ) ? 
		std::asin( std::clamp( shoulder_offset / r_proj.norm(), -1.0, 1.0 ) ) : 
		0.0;
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

	const Vec3d& p_wrist_center = Translation( wrist_center );
	const Vec3d& r_proj = ComputeDirection( wrist_center );
	double R = r_proj.norm();

	if ( R > epsilon )
	{
		if ( R + epsilon < std::abs( shoulder_offset_ ) )
		{
			presolution.joints = problem.seed;
			presolution.state = IKHeuristicState::Fail;
			return presolution;
		}

		const Vec3d& omega_b = base_joint->Axis();

		double alpha = ComputeAlpha( omega_b, reference_direction_, r_proj );
		double beta = ComputeBeta( shoulder_offset_, r_proj );

		auto candidates = EvaluateCandidates( problem.seed[0], alpha, beta );
		if ( !candidates.empty() )
		{
			presolution.joints = problem.seed;
			VecXd base_value( 1 );
			base_value[0] = candidates[0];
			GetGroup().SetGroupJoints( base_value, presolution.joints );
			presolution.state = IKHeuristicState::Success;
		}
		else
		{
			presolution.joints = problem.seed;
			presolution.state = IKHeuristicState::Fail;
		}
	}
	
	return presolution;
}

// ------------------------------------------------------------

std::vector< double > RevoluteBaseHeuristic::EvaluateCandidates(
	double seed,
	double alpha,
	double beta ) const
{
	const auto* base_joint = GetBaseJoint();
	const auto& limits = base_joint->GetLimits();

	double candidate1, candidate2;

	if ( shoulder_offset_ < epsilon )
	{
		candidate1 = WrapAngle( alpha );
		candidate2 = WrapAngle( alpha + M_PI );
	}
	else 
	{
		candidate1 = WrapAngle( alpha + M_PI - beta );
		candidate2 = WrapAngle( alpha + beta );
	}

	bool isvalid1 = limits.Within( candidate1 );
	bool isvalid2 = limits.Within( candidate2 );

	if ( isvalid1 && !isvalid2 )
	{
		return { candidate1 };
	}
	else if ( !isvalid1 && isvalid2 )
	{
		return { candidate2 };
	}
	else if ( !isvalid1 && !isvalid2 )
	{
		return {};
	}

	return std::abs( candidate1 - seed ) < std::abs( candidate2 - seed ) ?
	       std::vector< double >{ candidate1, candidate2 } :
	       std::vector< double >{ candidate2, candidate1 };
}

// ------------------------------------------------------------

}