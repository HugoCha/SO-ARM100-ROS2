#include "HybridSolver/WristAnalyzer.hpp"

#include "HybridSolver/WristModel.hpp"
#include "Model/JointChain.hpp"
#include "Model/Twist.hpp"
#include "Utils/KinematicsUtils.hpp"

#include <optional>

namespace SOArm100::Kinematics
{

// ------------------------------------------------------------

std::optional< const Vec3d > ComputeIntersection( const std::span< const JointConstPtr >& active_joints );
bool AxesIndependent( const std::span< const JointConstPtr >& active_joints );
const Mat4d ComputeTCPinWrist( const Vec3d& wrist_center, const Mat4d& home_configuration );

// ------------------------------------------------------------

std::optional< WristModel > WristAnalyzer::Analyze(
	const JointChain& joint_chain,
	const Mat4d& home_configuration )
{
	WristModel wrist;

	const auto& active_joints = joint_chain.GetActiveJoints();
	for ( int k = 1; k <= 3 && k <= active_joints.size(); ++k )
	{
		auto maybe_center = ComputeIntersection( active_joints.last( k ) );
		if ( !maybe_center )
			break;

		if ( !AxesIndependent( active_joints.last( k ) ) )
			break;

		wrist.active_joint_start = active_joints.size() - k;
		wrist.active_joint_count = k;
		wrist.center_at_home = *maybe_center;
	}

	if ( wrist.active_joint_count == 0 )
		return std::nullopt;

	wrist.tcp_in_wrist_at_home = ComputeTCPinWrist( wrist.center_at_home, home_configuration );
	wrist.tcp_in_wrist_at_home_inv = Inverse( wrist.tcp_in_wrist_at_home );
	wrist.type = static_cast< WristType >( wrist.active_joint_count );
	return wrist;
}

// ------------------------------------------------------------

std::optional< const Vec3d > ComputeIntersection( const std::span< const JointConstPtr >& active_joints )
{
	int n = active_joints.size();
	assert( n >= 1 && n <= 3 );

	Vec3d wrist_center;
	Eigen::Matrix< double, 9, 3 > omega_x;
	Eigen::Matrix< double, 9, 1 > opp_v;

	for ( int i = 0; i < n; ++i )
	{
		const auto& twist = active_joints[i]->GetTwist();
		omega_x.block< 3, 3 >( 3 * i, 0 ) = SkewMatrix( twist.GetAxis() );
		opp_v.segment< 3 >( 3 * i ) = -twist.GetLinear();
	}

	// Wrist center is the point Pwc such as omega_i x Pwc = -v_i
	// <=> || omega_i_x * Pwc + vi ||^2 = 0
	// <=> Solve least square equation : min( f(Pwc) )
	// with f( Pwc ) = || omega_i_x * Pwc + vi ||^2
	const int rows = 3 * n;

	wrist_center = omega_x.topRows( rows )
	               .colPivHouseholderQr()
	               .solve( opp_v.topRows( rows ) );

	double max_residual = 0.0;
	for ( int i = 0; i < n; ++i )
	{
		const Mat3d& omega_i_x = omega_x.block< 3, 3 >( 3 * i, 0 );
		const Vec3d& opp_v_i = opp_v.segment< 3 >( 3 * i );
		const Vec3d& residual = omega_i_x * wrist_center - opp_v_i;
		max_residual = std::max( max_residual, residual.norm() );
	}

	if ( max_residual > epsilon )
		return std::nullopt;

	return wrist_center;
}

// ------------------------------------------------------------

bool AxesIndependent( const std::span< const JointConstPtr >& active_joints )
{
	const int k = static_cast< int >( active_joints.size() );
	assert( k >= 1 && k <= 3 );

	if ( k == 1 )
		return active_joints[0]->GetTwist().GetAxis().norm() > epsilon;

	if ( k == 2 )
	{
		const Vec3d& w1 = active_joints[0]->GetTwist().GetAxis();
		const Vec3d& w2 = active_joints[1]->GetTwist().GetAxis();

		return w1.cross( w2 ).norm() > epsilon;
	}

	Mat3d W;
	for ( int i = 0; i < 3; ++i )
		W.col( i ) = active_joints[i]->GetTwist().GetAxis();

	return std::abs( W.determinant() ) > epsilon;
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