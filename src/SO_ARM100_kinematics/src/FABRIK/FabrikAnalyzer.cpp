#include "FABRIK/FabrikAnalyzer.hpp"

#include "Global.hpp"

#include "Model/Joint/JointChain.hpp"
#include "Model/Joint/JointGroup.hpp"

namespace SOArm100::Kinematics::Model
{

// ------------------------------------------------------------

std::optional< JointGroup > FabrikAnalyzer::Analyze(
	const JointChain& chain,
	const JointGroup& sub_group )
{
	if ( !JointGroup::IsConsistent( chain, sub_group ) ||
	     sub_group.Size() == 0 )
		return std::nullopt;

	std::vector< int > fabrik_indices;
	auto previous_joint_index = sub_group.FirstIndex();
	int sub_count = 1;
	auto joint = chain.GetActiveJoint( sub_group.FirstIndex() );
	auto it = ++sub_group.indices.begin();
	for ( ; it != sub_group.indices.end(); ++it )
	{
		if ( joint->IsRevolute() )
		{
			auto next_joint = chain.GetActiveJoint( *it );
			Vec3d axis = joint->Axis();
			Vec3d dir  = next_joint->Origin() - joint->Origin();

			if ( dir.norm() < epsilon || axis.cross( dir ).norm() < epsilon )
			{
				joint = chain.GetActiveJoint( *it );
				sub_count++;
				previous_joint_index = *it;
				continue;
			}
		}
		fabrik_indices.emplace_back( previous_joint_index );
		joint = chain.GetActiveJoint( *it );
		previous_joint_index = *it;
		sub_count = 1;
	}

	auto last_joint = chain.GetActiveJoint( previous_joint_index );
	if ( last_joint->IsPrismatic() )
	{
		fabrik_indices.emplace_back( previous_joint_index );
	}
	else
	{
		Vec3d p_tip = Translation( sub_group.tip_home );
		Vec3d axis = last_joint->Axis();
		Vec3d dir  = p_tip - last_joint->Origin();

		if ( dir.norm() > epsilon && axis.cross( dir ).norm() > epsilon )
			fabrik_indices.emplace_back( previous_joint_index );
	}

	if ( fabrik_indices.empty() )
		return std::nullopt;

	auto last_fabrik_idx = fabrik_indices.back();
	Mat4d fabrik_tip_home;
	if ( last_fabrik_idx != sub_group.LastIndex() )
	{
		fabrik_tip_home = chain.GetActiveJoint( sub_group.LastIndex() )->OriginTransform();
	}
	else
	{
		fabrik_tip_home = sub_group.tip_home;
	}

	return JointGroup{
	    fabrik_group_name,
	    std::set< int >( fabrik_indices.begin(), fabrik_indices.end() ),
	    fabrik_tip_home };
}

// ------------------------------------------------------------

std::optional< JointGroup > FabrikAnalyzer::Analyze(
	const JointChain& chain,
	const Mat4d& home,
	int start,
	int count )
{
	return Analyze(
		chain,
		JointGroup::CreateFromRange( "enumerate", start, count, home ) );
}

// ------------------------------------------------------------

std::optional< JointGroup > FabrikAnalyzer::Analyze(
	const JointChain& chain,
	const Mat4d& home )
{
	return Analyze( chain, home, 0, chain.GetActiveJointCount() );
}

// ------------------------------------------------------------

}