#include "ModelAnalyzer/WristAnalyzer.hpp"

#include "Global.hpp"

#include "Model/Joint/JointGroup.hpp"
#include "Model/Joint/JointChain.hpp"
#include "Utils/KinematicsUtils.hpp"

#include <algorithm>
#include <optional>

namespace SOArm100::Kinematics::Model
{

// ------------------------------------------------------------

const Mat4d ComputeTCPinWrist( const Vec3d& wrist_center, const Mat4d& home_configuration );

// ------------------------------------------------------------

std::optional< JointGroup > WristAnalyzer::Analyze(
	const JointChain& chain,
	const Mat4d& home )
{
	int wrist_start = 0;
	int wrist_count = 0;
	Vec3d wrist_center = Vec3d::Zero();

	const auto& active_joints = chain.GetActiveJoints();
	for ( int k = 1; k <= 3 && k <= active_joints.size(); ++k )
	{
		auto maybe_center = ComputeIntersection( active_joints.last( k ) );
		if ( !maybe_center )
			break;

		if ( !AxesIndependent( active_joints.last( k ) ) )
			break;

		wrist_start = active_joints.size() - k;
		wrist_count = k;
		wrist_center = *maybe_center;
	}

	if ( wrist_count == 0 )
		return std::nullopt;

	Mat4d wrist_tip = ComputeTCPinWrist( wrist_center, home );

	return WristJointGroup(
		wrist_start,
		wrist_count,
		wrist_tip );
}

// ------------------------------------------------------------

bool WristAnalyzer::CheckConsistency(
	const JointChain& joint_chain,
	const JointGroup& wrist_group )
{
	if ( !JointGroup::IsConsistent( joint_chain, wrist_group ) )
		return false;

	const int n_joints = joint_chain.GetActiveJointCount();
	const int n_joints_group = wrist_group.Size();

	if ( n_joints_group > std::min( 3, n_joints ) ||
	     wrist_group.name !=  wrist_name )
		return false;

	if ( !JointGroup::IsDense( wrist_group ) ||
	     wrist_group.LastIndex() != n_joints - 1 )
		return false;

	for ( int i = 0; i < n_joints_group; i++ )
		if ( !joint_chain.GetActiveJoint( wrist_group.Index( i ) )->IsRevolute() )
			return false;

	return true;
}

// ------------------------------------------------------------

const Mat4d ComputeTCPinWrist( const Vec3d& wrist_center, const Mat4d& home_configuration )
{
	Mat4d tcp_in_wrist = home_configuration;

	const auto& home_trans = Translation( home_configuration );
	const auto& home_rot = Rotation( home_configuration );
	const auto& home_rot_inv = home_rot.transpose();

	tcp_in_wrist.block< 3, 1 >( 0, 3 ) =
		home_rot_inv * ( home_trans - wrist_center );

	return tcp_in_wrist;
}

// ------------------------------------------------------------

}