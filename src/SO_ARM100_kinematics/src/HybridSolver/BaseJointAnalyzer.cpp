#include "HybridSolver/BaseJointAnalyzer.hpp"

#include "Global.hpp"
#include "HybridSolver/BaseJointModel.hpp"
#include "HybridSolver/WristModel.hpp"
#include "Joint/Joint.hpp"
#include "Joint/JointChain.hpp"
#include "Utils/KinematicsUtils.hpp"

#include <optional>

namespace SOArm100::Kinematics
{

// ------------------------------------------------------------

std::optional< BaseJointModel > BaseJointAnalyzer::Analyze(
	const JointChain& joint_chain,
	const std::optional< WristModel >& wrist_model )
{
	if ( !wrist_model || joint_chain.GetActiveJointCount() == wrist_model->active_joint_count )
		return std::nullopt;

	const auto& base_joint = joint_chain.GetActiveJoints()[0];
	if ( !base_joint->IsRevolute() )
		return std::nullopt;
	
	const Vec3d& base_origin = base_joint->Origin();
	const Vec3d& base_axis = base_joint->Axis();
	
	const Vec3d& wrist_center_at_home = wrist_model->center_at_home;
	const Vec3d r = wrist_center_at_home - base_origin;
	Vec3d reference_direction = r - r.dot( base_axis ) * base_axis;

	if ( reference_direction.norm() < epsilon  )
	{
		const auto* next_joint = joint_chain.GetNextJoint( base_joint );
		if ( next_joint )
		{
			const Vec3d& next_axis = next_joint->Axis();
			reference_direction = base_axis.cross( next_axis );
		}
	}
		
	if ( reference_direction.norm() < epsilon )
		return std::nullopt;

	return BaseJointModel{ reference_direction.normalized() };
}

// ------------------------------------------------------------

}