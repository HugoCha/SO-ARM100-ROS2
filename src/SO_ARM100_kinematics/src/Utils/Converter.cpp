#include "Utils/Converter.hpp"
#include "Global.hpp"

#include <Eigen/Dense>
#include <sstream>

namespace SOArm100::Kinematics
{

// ------------------------------------------------------------

VecXd ToVecXd( const std::span< const double >& vec )
{
	return Eigen::Map< const VecXd >( vec.data(), vec.size() );
}

// ------------------------------------------------------------

std::vector< double > ToStdVector( const VecXd& vec )
{
	return std::vector< double >( vec.data(), vec.data() + vec.size() );
}

// ------------------------------------------------------------

Mat4d ToTransformMatrix( const geometry_msgs::msg::Pose& pose_msg )
{
	Mat4d pose = Mat4d::Identity();

	pose.block< 3, 1 >( 0, 3 ) << pose_msg.position.x,
	    pose_msg.position.y,
	    pose_msg.position.z;

	Eigen::Quaterniond quaternion( pose_msg.orientation.w, pose_msg.orientation.x,
	                               pose_msg.orientation.y, pose_msg.orientation.z );

	pose.block< 3, 3 >( 0, 0 ) = quaternion.normalized().toRotationMatrix();

	return pose;
}

// ------------------------------------------------------------

Mat4d ToTransformMatrix( const Mat3d& rotation, const Vec3d& translation )
{
	Mat4d transform = Mat4d::Identity();
	transform.block< 3, 3 >( 0, 0 ) = rotation;
	transform.block< 3, 1 >( 0, 3 ) = translation;
	return transform;
}

// ------------------------------------------------------------

Mat4d ToTransformMatrix( const Mat3d& rotation )
{
	Mat4d transform = Mat4d::Identity();
	transform.block< 3, 3 >( 0, 0 ) = rotation;
	return transform;
}

// ------------------------------------------------------------

Mat4d ToTransformMatrix( const Vec3d& translation )
{
	Mat4d transform = Mat4d::Identity();
	transform.block< 3, 1 >( 0, 3 ) = translation;
	return transform;
}

// ------------------------------------------------------------

geometry_msgs::msg::Pose ToPoseMsg( const Mat4d& matrix )
{
	geometry_msgs::msg::Pose pose_msg;

	pose_msg.position.x = matrix( 0, 3 );
	pose_msg.position.y = matrix( 1, 3 );
	pose_msg.position.z = matrix( 2, 3 );

	Eigen::Quaterniond quaternion( matrix.block< 3, 3 >( 0, 0 ) );

	pose_msg.orientation.x = quaternion.x();
	pose_msg.orientation.y = quaternion.y();
	pose_msg.orientation.z = quaternion.z();
	pose_msg.orientation.w = quaternion.w();

	return pose_msg;
}

// ------------------------------------------------------------

const std::string ToString( const geometry_msgs::msg::Pose& pose )
{
	std::ostringstream oss;

	// Format the position (x, y, z)
	oss << "Position: ("
	    << pose.position.x << ", "
	    << pose.position.y << ", "
	    << pose.position.z << ") ";

	// Format the orientation (x, y, z, w)
	oss << "Orientation: ("
	    << pose.orientation.x << ", "
	    << pose.orientation.y << ", "
	    << pose.orientation.z << ", "
	    << pose.orientation.w << ")";

	return oss.str();
}

// ------------------------------------------------------------

}
