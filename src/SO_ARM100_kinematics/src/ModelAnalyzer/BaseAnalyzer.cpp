#include "ModelAnalyzer/BaseAnalyzer.hpp"

#include "Global.hpp"
#include "Model/JointChain.hpp"
#include "Model/JointGroup.hpp"
#include "Utils/KinematicsUtils.hpp"

#include <optional>

namespace SOArm100::Kinematics::Model
{

// ------------------------------------------------------------

std::optional< JointGroup > BaseAnalyzer::Analyze(
	const JointChain& joint_chain,
	const Mat4d& home,
	const std::optional< JointGroup >& wrist_group  )
{
	if ( !wrist_group || 
		 joint_chain.GetActiveJointCount() <= 1 )
		return std::nullopt;

	auto base_joint = joint_chain.GetActiveJoint( 0 );

	std::string base_name;

	const auto& wrist_home = wrist_group->tip_home;
	const auto& wrist_center = home * Inverse( wrist_home );

	if ( base_joint->IsRevolute() )
	{
		return RevoluteBaseJointGroup( wrist_center );
	}
	else if ( base_joint->IsPrismatic() )
	{
		return PrismaticBaseJointGroup( wrist_center );
	}

	return std::nullopt;
}

// ------------------------------------------------------------

bool BaseAnalyzer::CheckConsistency( 	
	const JointChain& joint_chain,
	const JointGroup& base_group )
{
	if ( !JointGroup::IsConsistent( joint_chain, base_group ) )
		return false;

	bool is_consistent = true;
	
	is_consistent |= base_group.name == revolute_base_name && joint_chain.GetActiveJoint( 0 )->IsRevolute();
	is_consistent |= base_group.name == prismatic_base_name && joint_chain.GetActiveJoint( 0 )->IsPrismatic();
	is_consistent &= base_group.Size() == 1 && base_group.Index( 0 ) == 0;

	return is_consistent;
}

// ------------------------------------------------------------

}