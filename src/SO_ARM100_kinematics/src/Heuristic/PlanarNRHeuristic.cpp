#include "Heuristic/PlanarNRHeuristic.hpp"

#include "Global.hpp"

#include "Heuristic/IKPresolution.hpp"
#include "Heuristic/Planar1RHeuristic.hpp"
#include "Heuristic/Planar2RHeuristic.hpp"
#include "Heuristic/PlanarCCDHeuristic.hpp"
#include "Model/IKJointGroupModelBase.hpp"
#include "Model/Joint/JointGroup.hpp"
#include "Solver/IKProblem.hpp"
#include "Solver/IKRunContext.hpp"
#include "Utils/KinematicsUtils.hpp"

#include <cmath>
#include <memory>

namespace SOArm100::Kinematics::Heuristic
{

// ------------------------------------------------------------

PlanarNRHeuristic::PlanarNRHeuristic(
	Model::KinematicModelConstPtr model,
	Model::JointGroup planar_group ) :
	Model::IKJointGroupModelBase( model, planar_group ),
	planar_heuristic_( std::move( InitializePlanarHeuristic( model, planar_group ) ) )
{
}

// ------------------------------------------------------------

IKPresolution PlanarNRHeuristic::Presolve(
	const Solver::IKProblem& problem,
	const Solver::IKRunContext& context ) const
{
	return planar_heuristic_->Presolve( problem, context );
}

// ------------------------------------------------------------

std::unique_ptr< IIKHeuristic > PlanarNRHeuristic::InitializePlanarHeuristic(
	Model::KinematicModelConstPtr model,
	Model::JointGroup planar_group )
{
	auto is_last_group_joint_at_home = [&]() -> bool {
										   const auto& chain = model->GetChain();
										   const auto& last_joint = chain->GetActiveJoint( planar_group.LastIndex() );
										   Vec3d p_tip = Translation( planar_group.tip_home );

										   return last_joint->Origin().isApprox( p_tip, epsilon );
									   };

	if ( planar_group.Size() == 1 ||
	     ( planar_group.Size() == 2 && is_last_group_joint_at_home() ) )
	{
		return std::make_unique< Planar1RHeuristic >( model, planar_group );
	}
	if ( planar_group.Size() == 2 ||
	     ( planar_group.Size() == 3 && is_last_group_joint_at_home() ) )
	{
		return std::make_unique< Planar2RHeuristic >( model, planar_group );
	}
	return std::make_unique< PlanarCCDHeuristic >( model, planar_group );
}

// ------------------------------------------------------------

}