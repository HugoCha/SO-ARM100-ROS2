#include "HybridKinematicsSolver.hpp"

#include "Converter.hpp"
#include "KinematicsUtils.hpp"
#include "MatrixExponential.hpp"
#include "Twist.hpp"

namespace SOArm100::Kinematics
{

// ------------------------------------------------------------

HybridKinematicsSolver::HybridKinematicsSolver()
{
}

// ------------------------------------------------------------

HybridKinematicsSolver::~HybridKinematicsSolver()
{
}

// ------------------------------------------------------------

bool HybridKinematicsSolver::InverseKinematic(
	const geometry_msgs::msg::Pose& target_pose,
	std::vector< double >& joint_angles )
{
	joint_angles.clear();

	return false;
}

// ------------------------------------------------------------

Mat4d HybridKinematicsSolver::ComputeWristCenterPose( const geometry_msgs::msg::Pose& target_pose )
{
	Mat4d target = ToMat4d( target_pose );

	Twist tool0_in_wrist_twist = twists_.back();
	Mat4d tool0_in_wrist = MatrixExponential( tool0_in_wrist_twist, 0.0 ) * home_configuration_;

	Mat3d tcp_orientation = Rotation( target );
	Vec3d tcp_position = Translation( target );

	Mat4d wrist_in_base = target;
	Vec3d wrist_position = tcp_position - tcp_orientation * Translation( tool0_in_wrist );
	wrist_in_base.block< 3, 1 >( 0, 3 ) = wrist_position;

	return wrist_in_base;
}

// ------------------------------------------------------------

double HybridKinematicsSolver::ComputeBaseJoint( const Mat4d& wrist_pose )
{
	double xw = wrist_pose( 0, 3 );
	double yw = wrist_pose( 1, 3 );

	return atan2( yw, xw );
}

// ------------------------------------------------------------

Mat4d HybridKinematicsSolver::ComputeShoulderPose( double base_joint )
{
	Twist base_to_shoulder = twists_.front();
	return MatrixExponential( base_to_shoulder, base_joint );
}

// ------------------------------------------------------------

std::vector< double > HybridKinematicsSolver::ComputeIntermediateJoints( const Mat4d& wrist_pose )
{
	return {};
}

// ------------------------------------------------------------

std::vector< double > HybridKinematicsSolver::ComputeWristJoints(
	const Mat4d& wrist_in_base,
	const Mat4d& target_pose )
{
	Mat4d tcp_in_wrist = wrist_in_base * target_pose;
	double q_wrist1 = atan2( tcp_in_wrist( 1, 0 ), tcp_in_wrist( 0, 0 ));
	double q_wrist2 = atan2( tcp_in_wrist( 2, 1 ), tcp_in_wrist( 2, 2 ));
	return { q_wrist1, q_wrist2 };
}

// ------------------------------------------------------------

} // namespace SOArm100::Kinematics
