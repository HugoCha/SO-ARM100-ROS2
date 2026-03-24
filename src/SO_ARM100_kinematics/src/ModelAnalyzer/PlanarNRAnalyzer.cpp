#include "ModelAnalyzer/PlanarNRAnalyzer.hpp"

#include "Global.hpp"

#include "Model/JointChain.hpp"
#include "Model/JointGroup.hpp"

#include <optional>

namespace SOArm100::Kinematics::Model
{

// ------------------------------------------------------------

Vec3d GetJointAxis( JointChain chain, int index );
Mat4d GetJointOriginTransform( JointChain chain, int index );

// ------------------------------------------------------------

std::optional< JointGroup > PlanarNRAnalyzer::Analyze(
	const JointChain& chain,
	const Mat4d& home,
	int start_idx,
	int N )
{
	const int n_joints = chain.GetActiveJointCount();

	if ( start_idx < 0 || start_idx + N > n_joints )
		return std::nullopt;
	
	auto axis = GetJointAxis( chain, start_idx );

	for ( int i = start_idx + 1; i < N; i++ )
	{
		if ( axis != GetJointAxis( chain, i ) )
			return std::nullopt;
	}

	Mat4d planar_home;

	if ( start_idx + N >= n_joints )
	{
		planar_home = home;
	}
	else
	{
		planar_home = GetJointOriginTransform( chain, start_idx + N );
	}

	return PlanarNRJointGroup( start_idx, N, planar_home );
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