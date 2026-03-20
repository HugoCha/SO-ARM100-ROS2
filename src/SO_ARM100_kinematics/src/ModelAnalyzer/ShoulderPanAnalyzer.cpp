#include "ModelAnalyzer/ShoulderPanAnalyzer.hpp"

#include "Global.hpp"
#include "Model/JointChain.hpp"
#include "Model/JointGroup.hpp"
#include "Utils/KinematicsUtils.hpp"

#include <optional>

namespace SOArm100::Kinematics::Model
{

// ------------------------------------------------------------

std::optional< JointGroup > ShoulderPanAnalyzer::Analyze(
	const JointChain& joint_chain,
	const Mat4d& home,
	const std::optional< JointGroup >& wrist_group  )
{
	if ( !wrist_group || joint_chain.GetActiveJointCount() <= 1 )
		return std::nullopt;

	auto base_joint = joint_chain.GetActiveJoint( 0 );
	if ( !base_joint->IsRevolute() )
		return std::nullopt;

	const auto& wrist_home = wrist_group->tip_home;
	const auto& wrist_center = home * Inverse( wrist_home );

	return JointGroup::CreateFromRange( 
		"shoulder_pan", 
		JointGroupType::ShoulderPan,
		0, 
		1, 
		wrist_center );
}

// ------------------------------------------------------------

}