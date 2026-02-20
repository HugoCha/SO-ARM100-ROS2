#include "HybridSolver/BaseJointAnalyzer.hpp"

#include "Global.hpp"
#include "HybridSolver/BaseJointModel.hpp"
#include "HybridSolver/WristModel.hpp"
#include "Joint/JointChain.hpp"
#include "Joint/Twist.hpp"
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

	const auto& base_twist = joint_chain.GetActiveJoints()[0]->GetTwist();

	const Vec3d& wrist_center_at_home = wrist_model->center_at_home;
	const Vec3d& omega = base_twist.GetAxis();
	const Vec3d r = wrist_center_at_home - base_twist.GetLinear();
	Vec3d reference_direction = r - r.dot( omega ) * omega;

	if ( reference_direction.norm() < epsilon )
		return std::nullopt;

	return BaseJointModel{ reference_direction.normalized() };
}

// ------------------------------------------------------------

}