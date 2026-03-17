#include "HybridSolver/BaseJointAnalyzer.hpp"

#include "Global.hpp"
#include "HybridSolver/BaseJointModel.hpp"
#include "HybridSolver/WristModel.hpp"
#include "Model/JointChain.hpp"

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

	const auto& wrist_at_home = wrist_model->center_at_home;

	const Vec3d r = wrist_at_home - base_origin;
	Vec3d reference_direction = r - r.dot( base_axis ) * base_axis;

	if ( reference_direction.norm() < epsilon )
	{
		auto wrist_joint = joint_chain.GetActiveJoint( wrist_model->active_joint_start );
		auto previous_wrist_joint = joint_chain.GetPreviousJoint( wrist_joint );

		if ( !previous_wrist_joint )
			return std::nullopt;

		const auto& previous_wrist_joint_axis = previous_wrist_joint->Axis();
		reference_direction = base_axis.cross( previous_wrist_joint_axis );

		if ( reference_direction.norm() < epsilon )
			return std::nullopt;
	}

	return BaseJointModel{ reference_direction.normalized() };
}

// ------------------------------------------------------------

}