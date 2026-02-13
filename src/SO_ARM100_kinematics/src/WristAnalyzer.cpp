#include "WristAnalyzer.hpp"

#include "KinematicsUtils.hpp"
#include "Twist.hpp"

#include <moveit/robot_model/joint_model.hpp>
#include <optional>

namespace SOArm100::Kinematics
{

// ------------------------------------------------------------

std::optional< const Vec3d > ComputeIntersection( const std::span< TwistConstPtr >& twists );
bool AxesIndependent( const std::span< TwistConstPtr >& twists );
const Mat4d ComputeTCPinWrist( const Vec3d& wrist_center, const Mat4d& home_configuration );

// ------------------------------------------------------------

std::optional< WristModel > WristAnalyzer::Analyze(
	const std::span< const moveit::core::JointModel* const >& joint_models,
	const std::span< TwistConstPtr >& twists,
	const Mat4d& home_configuration )
{
	WristModel wrist;

	for ( int k = 1; k <= 3 && k <= twists.size(); ++k )
	{
		auto maybe_center = ComputeIntersection( twists.last( k ) );
		if ( !maybe_center )
			break;

		if ( !AxesIndependent( twists.last( k ) ) )
			break;

		wrist.start_index = twists.size() - k - 1;
		wrist.count = k;
		wrist.center_at_home = *maybe_center;
	}

	if ( wrist.count == 0 )
		return std::nullopt;

	wrist.wrist_joints = joint_models.last( wrist.count );
	wrist.twists = twists.last( wrist.count );
    wrist.tcp_in_wrist_at_home = ComputeTCPinWrist( wrist.center_at_home, home_configuration );
	wrist.tcp_in_wrist_at_home_inv = Inverse( wrist.tcp_in_wrist_at_home );
	wrist.type = static_cast< WristType >( wrist.count );
	return wrist;
}

// ------------------------------------------------------------

std::optional< const Vec3d > ComputeIntersection( const std::span< const Twist >& twists )
{
	int n = twists.size();
	assert( n >= 1 && n <= 3 );

	Vec3d wrist_center;
	Eigen::Matrix< double, 9, 3 > omega_x;
	Eigen::Matrix< double, 9, 1 > opp_v;

	for ( int i = 0; i < n; ++i )
	{
		omega_x.block< 3, 3 >( 3 * i, 0 ) = SkewMatrix( twists[i].GetAxis() );
		opp_v.segment< 3 >( 3 * i ) = -twists[i].GetLinear();
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
		const Vec3d& omega_i_x = omega_x.block< 3, 3 >( 3 * i, 0 );
		const Vec3d& opp_v_i = opp_v.segment< 3 >( 3 * i );
		const Vec3d& residual = omega_i_x * wrist_center - opp_v_i;
		max_residual = std::max( max_residual, residual.norm() );
	}

	if ( max_residual > epsilon )
		return std::nullopt;

	return wrist_center;
}

// ------------------------------------------------------------

bool AxesIndependent( std::span< const Twist > twists )
{
	const int k = static_cast< int >( twists.size() );
	assert( k >= 1 && k <= 3 );

	if ( k == 1 )
		return twists[0].GetAxis().norm() > epsilon;

	if ( k == 2 )
	{
		const Vec3d& w1 = twists[0].GetAxis();
		const Vec3d& w2 = twists[1].GetAxis();

		return w1.cross( w2 ).norm() > epsilon;
	}

	Mat3d W;
	for ( int i = 0; i < 3; ++i )
		W.col( i ) = twists[i].GetAxis();

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