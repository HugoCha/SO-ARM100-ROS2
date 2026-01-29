#include "Converter.hpp"

#include <Eigen/Geometry>

namespace SOArm100::Kinematics
{

// ------------------------------------------------------------

VecXd ToVecXd( const std::vector< double >& vec )
{
	VecXd eigen_vec( vec.size());
	for ( size_t i = 0; i < vec.size(); ++i )
	{
		eigen_vec( i ) = vec[i];
	}
	return eigen_vec;
}

// ------------------------------------------------------------

std::vector< double > ToStdVector( const VecXd& vec )
{
	return std::vector< double >( vec.data(), vec.data() + vec.size());
}

// ------------------------------------------------------------

Mat4d ToMat4d( const geometry_msgs::msg::Pose& pose_msg )
{
	Mat4d pose = Mat4d::Identity();

	pose( 0, 3 ) = pose_msg.position.x;
	pose( 1, 3 ) = pose_msg.position.y;
	pose( 2, 3 ) = pose_msg.position.z;

	Eigen::Quaterniond quaternion( pose_msg.orientation.x, pose_msg.orientation.y,
	                               pose_msg.orientation.z, pose_msg.orientation.w );
	quaternion.normalize();
	pose.block< 3, 3 >( 0, 0 ) = quaternion.toRotationMatrix();

	return pose;
}

// ------------------------------------------------------------

geometry_msgs::msg::Pose  ToPoseMsg( const Mat4d& matrix )
{
	geometry_msgs::msg::Pose pose_msg;

	pose_msg.position.x = matrix( 0, 3 );
	pose_msg.position.y = matrix( 1, 3 );
	pose_msg.position.z = matrix( 2, 3 );

	Mat3d rotation_matrix = matrix.block< 3, 3 >( 0, 0 );
	Eigen::Quaterniond quaternion( rotation_matrix );

	pose_msg.orientation.x = quaternion.x();
	pose_msg.orientation.y = quaternion.y();
	pose_msg.orientation.z = quaternion.z();
	pose_msg.orientation.w = quaternion.w();

	return pose_msg;
}

// ------------------------------------------------------------

}
