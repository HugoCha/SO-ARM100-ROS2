#include "HybridSolver/BaseJointAnalyzer.hpp"

#include "Global.hpp"
#include "HybridSolver/BaseJointModel.hpp"
#include "HybridSolver/WristModel.hpp"
#include "Joint/Joint.hpp"
#include "Joint/JointChain.hpp"
#include <optional>

namespace SOArm100::Kinematics
{

// ------------------------------------------------------------

std::optional< BaseJointModel > BaseJointAnalyzer::Analyze(
	const JointChain& joint_chain,
	const std::optional< WristModel >& wrist_model )
{
	if ( !wrist_model || joint_chain.GetActiveJointCount() <= 1 )
		return std::nullopt;

	auto base_joint = joint_chain.GetActiveJoint( 0 );
	const auto& base_axis = base_joint->Axis();
	const auto& base_origin = base_joint->Origin();

	auto next_joint = joint_chain.GetNextJoint( base_joint );
	const auto& next_axis = next_joint->Axis();
	const auto& next_origin = next_joint->Origin();

	const Vec3d r = next_origin - base_origin;
	Vec3d reference_direction = r - r.dot( base_axis ) * base_axis;

	if ( reference_direction.norm() < epsilon )
	{
		reference_direction = base_axis.cross( next_axis );

		if ( reference_direction.norm() < epsilon )
			return std::nullopt;
	}

	return BaseJointModel{ reference_direction.normalized() };
}

// ------------------------------------------------------------

}