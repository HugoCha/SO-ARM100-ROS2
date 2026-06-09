#include "Heuristic/PrismaticBaseHeuristic.hpp"

#include "Global.hpp"

#include "Heuristic/IKHeuristicState.hpp"
#include "Heuristic/IKPresolution.hpp"
#include "Model/IKJointGroupModelBase.hpp"
#include "Model/Joint/JointGroup.hpp"
#include "Solver/IKProblem.hpp"
#include "Utils/MathUtils.hpp"

namespace SOArm100::Kinematics::Heuristic
{

// ------------------------------------------------------------

PrismaticBaseHeuristic::PrismaticBaseHeuristic(
	Model::KinematicModelConstPtr model,
	const Model::JointGroup& prismatic_base_group ) :
	Model::IKJointGroupModelBase( model, prismatic_base_group )
{
}

// ------------------------------------------------------------

IKPresolution PrismaticBaseHeuristic::Presolve(
	const Solver::IKProblem& problem,
	const Solver::IKRunContext& context ) const
{
	IKPresolution presolution{ problem.seed, IKHeuristicState::Fail };

	auto base_joint = GetBaseJoint();

	auto wrist_center = ComputeGroupLocalTarget( problem.seed, problem.target );

    const Vec3d& origin = base_joint->Origin();
    const Vec3d& axis   = base_joint->Axis();

	const Vec3d& p_wrist_center = Translation( wrist_center );

    Vec3d wc_proj = ProjectPointOnAxis( p_wrist_center - origin, Vec3d::Zero(), axis );

    double value = wc_proj.norm();
    Vec1d clamp_value( base_joint->GetLimits().Clamp( value ) );
    
    presolution.error = std::abs( clamp_value[0] - value );
    
    if ( base_joint->GetLimits().Within( value ) )
    {
        presolution.state = IKHeuristicState::Success;
    }
    else
    {
        presolution.state = ( presolution.error < 2 * problem.tolerance ) ? IKHeuristicState::PartialSuccess : IKHeuristicState::Fail;
    }

    GetGroup().SetGroupJoints( clamp_value, presolution.joints );
	return presolution;
}

// ------------------------------------------------------------

const Model::Joint* PrismaticBaseHeuristic::GetBaseJoint() const
{
	return GetChain()->GetActiveJoint( GetGroup().FirstIndex() ).get();
}

// ------------------------------------------------------------

}