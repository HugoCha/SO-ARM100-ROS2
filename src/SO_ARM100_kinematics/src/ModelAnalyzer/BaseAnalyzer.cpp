#include "ModelAnalyzer/BaseAnalyzer.hpp"

#include "Global.hpp"

#include "Model/Joint/JointChain.hpp"
#include "Model/Joint/JointGroup.hpp"
#include "Utils/KinematicsUtils.hpp"

#include <optional>

namespace SOArm100::Kinematics::Model
{

// ------------------------------------------------------------

std::optional< JointGroup > BaseAnalyzer::Analyze(
	const JointChain& joint_chain,
	const Mat4d& home,
	const std::optional< PlanarNRJointGroup >& planar_group,
	const std::optional< WristJointGroup >& wrist_group  )
{
	if ( !wrist_group ||
	     joint_chain.GetActiveJointCount() <= 1 )
		return std::nullopt;

	if ( !planar_group )
	{
		if ( wrist_group->FirstIndex() != 1 )
			return std::nullopt;
	}
	else
	{
		if ( planar_group->FirstIndex() == 0 )
			return std::nullopt;
	}

	auto base_joint = joint_chain.GetActiveJoint( 0 );
	Mat4d tip_home = wrist_group->GetWristCenter();

	if ( base_joint->IsRevolute() )
	{
		Vec3d p_tip_home = Translation( tip_home );

		// If wrist center at home is along base axis
		if ( p_tip_home.cross( base_joint->Axis() ).norm() < epsilon )
			return std::nullopt;

		return RevoluteBaseJointGroup( tip_home );
	}
	else if ( base_joint->IsPrismatic() )
	{
		return PrismaticBaseJointGroup( tip_home );
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

	bool is_consistent = false;

	is_consistent |= base_group.name == revolute_base_name && joint_chain.GetActiveJoint( 0 )->IsRevolute();
	is_consistent |= base_group.name == prismatic_base_name && joint_chain.GetActiveJoint( 0 )->IsPrismatic();
	is_consistent &= base_group.Size() == 1 && base_group.Index( 0 ) == 0;

	return is_consistent;
}

// ------------------------------------------------------------

}