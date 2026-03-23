#include "ModelAnalyzer/PlanarNRAnalyzer.hpp"

#include "Global.hpp"
#include "Model/JointChain.hpp"
#include "Model/JointGroup.hpp"
#include <vector>

namespace SOArm100::Kinematics::Model
{

// ------------------------------------------------------------

Vec3d GetJointAxis( JointChain chain, int index );
Mat4d GetJointOriginTransform( JointChain chain, int index );

// ------------------------------------------------------------

std::vector< JointGroup > PlanarNRAnalyzer::Analyze(
	const JointChain& chain,
	const Mat4d& home,
	int start_idx,
	int count )
{
	const int n_joints = chain.GetActiveJointCount();

	if ( start_idx < 0 || count <= 1 || start_idx + count > n_joints )
		return {}
	;

	std::vector< JointGroup > planar_groups;
	auto active_joints = chain.GetActiveJoints();

	int sub_planar_start = start_idx;
	int sub_planar_count = 1;
	auto axis = GetJointAxis( chain, start_idx );

	while ( sub_planar_start < count )
	{
		if ( sub_planar_start + sub_planar_count < count &&
		     axis == GetJointAxis( chain, sub_planar_start + sub_planar_count ) )
		{
			sub_planar_count++;
		}
		else
		{
			if ( sub_planar_count >= 2 )
			{
				Mat4d planar_group_home;
				if ( start_idx + count >= n_joints )
				{
					planar_group_home = home;
				}
				else
				{
					planar_group_home = GetJointOriginTransform( chain, sub_planar_start + sub_planar_count );
				}

				auto sub_planar_group = PlanarNRJointGroup(
					planar_groups.size(),
					sub_planar_start,
					sub_planar_count -  1,
					planar_group_home );

				planar_groups.emplace_back( sub_planar_group );
			}

			sub_planar_start += sub_planar_count;
			sub_planar_count = 1;
			if ( sub_planar_start < n_joints )
				axis = GetJointAxis( chain, sub_planar_start );
		}
	}

	return planar_groups;
}

// ------------------------------------------------------------

std::vector< JointGroup > PlanarNRAnalyzer::Analyze(
	const JointChain& chain,
	const Mat4d& home )
{
	return Analyze( chain, home, 0, chain.GetActiveJointCount() );
}

// ------------------------------------------------------------

bool PlanarNRAnalyzer::CheckConsistency(
	const JointChain& chain,
	const JointGroup& planar_group )
{
	if ( !JointGroup::IsConsistent( chain, planar_group ) )
		return false;

	const int n_joints = chain.GetActiveJointCount();
	const int n_planar_joints = planar_group.Size();

	if ( !planar_group.name.contains( planarNR_name ) ||
	     n_planar_joints <= 1 )
		return false;

	if ( !JointGroup::IsDense( planar_group ) )
		return false;

	auto axis = chain.GetActiveJoint( planar_group.Index( 0 ) )->Axis();

	for ( int i = 1; i < n_planar_joints; i++ )
	{
		if ( chain.GetActiveJoint( planar_group.Index( i ) )->Axis() != axis )
			return false;
	}

	return true;
}

// ------------------------------------------------------------

Vec3d GetJointAxis( JointChain chain, int index )
{
	auto joint = chain.GetActiveJoint( index );
	if ( !joint )
		return Vec3d::Zero();
	return joint->Axis();
}

// ------------------------------------------------------------

Mat4d GetJointOriginTransform( JointChain chain, int index )
{
	return chain.GetActiveJoint( index )->OriginTransform();
}

// ------------------------------------------------------------

}