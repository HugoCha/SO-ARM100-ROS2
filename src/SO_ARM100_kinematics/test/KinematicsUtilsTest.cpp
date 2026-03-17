#include "Global.hpp"

#include "Model/JointChain.hpp"
#include "Model/Limits.hpp"
#include "Model/Twist.hpp"
#include "RobotModelTestData.hpp"
#include "Utils/KinematicsUtils.hpp"

#include <gtest/gtest.h>
#include <ostream>

namespace SOArm100::Kinematics::Test
{

// ------------------------------------------------------------
// ------------------------------------------------------------

class KinematicsUtilsTest : public ::testing::Test {
protected:
void SetUp() override
{
}

void TearDown() override
{
}
};

// ------------------------------------------------------------
// ------------------------------------------------------------

TEST_F( KinematicsUtilsTest, SkewMatrixTest )
{
	Eigen::Vector3d vec( 1.0, 2.0, 3.0 );
	Eigen::Matrix3d expected;
	expected << 0, -3.0, 2.0,
	    3.0, 0, -1.0,
	    -2.0, 1.0, 0;

	Eigen::Matrix3d result = SOArm100::Kinematics::SkewMatrix( vec );

	ASSERT_TRUE( result.isApprox( expected ) );
}

// ------------------------------------------------------------

TEST_F( KinematicsUtilsTest, InverseTransform )
{
	Mat4d transform = Mat4d::Identity();
	transform.block< 3, 3 >( 0, 0 ) = Eigen::AngleAxisd( M_PI / 2, Eigen::Vector3d::UnitZ() ).toRotationMatrix();
	transform.block< 3, 1 >( 0, 3 ) = Vec3d( 1.0, 2.0, 3.0 );

	Mat4d inverse = Inverse( transform );

	// Verify that transform * inverse = identity
	Mat4d identity = inverse * transform;
	EXPECT_TRUE( identity.isApprox( Mat4d::Identity(), 1e-6 ) ) <<
	    "Transform=\n" << transform <<
	    "\nInverse=\n" << inverse <<
	    "\nInverse * Transform=\n" << identity << std::endl;
}

// ------------------------------------------------------------

TEST_F( KinematicsUtilsTest, AdjointIdentityTransform )
{
	Mat4d identity = Mat4d::Identity();
	Mat6d adjoint;

	Adjoint( identity, adjoint );

	// Adjoint of identity transform should be identity
	Mat6d expected = Mat6d::Identity();
	EXPECT_TRUE( adjoint.isApprox( expected, 1e-6 ) ) << "Adjoint of identity transform should be identity";
}

// ------------------------------------------------------------

TEST_F( KinematicsUtilsTest, AdjointNonIdentityTransform )
{
	Mat4d transform = Mat4d::Identity();
	transform( 0, 3 ) = 1.0; // Translation in x
	transform( 1, 3 ) = 2.0; // Translation in y
	transform( 2, 3 ) = 3.0; // Translation in z

	Mat6d adjoint;
	Adjoint( transform, adjoint );

	// Verify the structure of the adjoint matrix
	EXPECT_TRUE( ( adjoint.block< 3, 3 >( 0, 0 ) ).isApprox( Mat3d::Identity(), 1e-6 ) ) << "Top-left block should be identity";
	EXPECT_TRUE( ( adjoint.block< 3, 3 >( 3, 3 ) ).isApprox( Mat3d::Identity(), 1e-6 ) ) << "Bottom-right block should be identity";
}

// ------------------------------------------------------------

TEST_F( KinematicsUtilsTest, SpaceJacobianSingleTwist )
{
	JointChain joint_chain( 1 );
	Twist twist( Vec3d( 0, 0, 1 ), Vec3d( 0, 0, 0 ) );
	Link link( Mat4d::Identity(), 0 );
	Limits limits( -M_PI, M_PI );
	joint_chain.Add( twist, link, limits ); // Pure rotation around z-axis

	VecXd joint_angles( 1 );
	joint_angles << 0.0;

	MatXd jacobian( 6, 1 );
	SpaceJacobian( joint_chain, joint_angles, jacobian );

	// Expected Jacobian for a single twist (pure rotation around z-axis)
	Vec6d expected_jacobian;
	expected_jacobian << 0, 0, 1, 0, 0, 0;
	EXPECT_TRUE( jacobian.col( 0 ).isApprox( expected_jacobian, 1e-6 ) ) << "Jacobian for single twist should match expected values";
}

// ------------------------------------------------------------

TEST_F( KinematicsUtilsTest, SpaceJacobianMultipleTwists )
{
	JointChain joint_chain( 2 );

	joint_chain.Add(
		{ Vec3d( 0, 0, 1 ), Vec3d( 0, 0, 0 ) },
		{},
		{ -M_PI, M_PI } );

	joint_chain.Add(
		{ Vec3d( 0, 1, 0 ), Vec3d( 0, 0, 0 ) },
		{},
		{ -M_PI, M_PI } );

	VecXd joint_angles( 2 );
	joint_angles << 0.0, 0.0;

	MatXd jacobian( 6, 2 );
	SpaceJacobian( joint_chain, joint_angles, jacobian );

	// Expected Jacobian for two twists (pure rotations around z and y axes)
	Vec6d expected_jacobian_col1, expected_jacobian_col2;
	expected_jacobian_col1 << 0, 0, 1, 0, 0, 0;
	expected_jacobian_col2 << 0, 1, 0, 0, 0, 0;
	EXPECT_TRUE( jacobian.col( 0 ).isApprox( expected_jacobian_col1, 1e-6 ) ) << "First column of Jacobian should match expected values";
	EXPECT_TRUE( jacobian.col( 1 ).isApprox( expected_jacobian_col2, 1e-6 ) ) << "Second column of Jacobian should match expected values";
}

// ------------------------------------------------------------

TEST_F( KinematicsUtilsTest, SpaceJacobianRevoluteOnlyRobotTwist )
{
	const auto& joint_chain = Data::GetRevoluteOnlyRobotJointChain();
	VecXd joint_angles( 3 );
	joint_angles << 1.5708, 0.7854, 0.3927;

	MatXd geom_jacobian = Data::GetRevoluteOnlyRobotJacobian(
		joint_angles[0],
		joint_angles[1],
		joint_angles[2] );
	Mat4d T0e = Data::GetRevoluteOnlyRobotTransform( joint_angles[0], joint_angles[1], joint_angles[2] );
	Mat6d adjoint_T0e;
	Mat4d Tee = Mat4d::Identity();
	Tee.block< 3, 1 >( 0, 3 ) = Translation( T0e );
	Adjoint( Tee, adjoint_T0e );

	MatXd expected_space_jacobian = adjoint_T0e * geom_jacobian;

	MatXd space_jacobian( 6, 3 );
	SpaceJacobian( joint_chain, joint_angles, space_jacobian );

	EXPECT_TRUE( expected_space_jacobian.isApprox( space_jacobian ) )
	    << "Expected = \n" << expected_space_jacobian << std::endl
	    << "Actual = \n" << space_jacobian << std::endl;
}

// ------------------------------------------------------------

TEST_F( KinematicsUtilsTest, PseudoInverseIdentityMatrix )
{
	MatXd identity = MatXd::Identity( 6, 6 );
	MatXd pseudo_inverse;

	PseudoInverse( identity, pseudo_inverse );

	// Pseudo-inverse of identity matrix should be identity
	EXPECT_TRUE( pseudo_inverse.isApprox( identity, 1e-6 ) ) << "Pseudo-inverse of identity matrix should be identity";
}

// ------------------------------------------------------------

TEST_F( KinematicsUtilsTest, PseudoInverseNonSquareMatrix )
{
	MatXd matrix( 6, 3 );
	matrix << 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0;
	MatXd pseudo_inverse;

	PseudoInverse( matrix, pseudo_inverse );

	// Verify pseudo-inverse has the correct dimensions
	EXPECT_EQ( pseudo_inverse.rows(), 3 ) << "Pseudo-inverse should have 3 rows";
	EXPECT_EQ( pseudo_inverse.cols(), 6 ) << "Pseudo-inverse should have 6 columns";
}

// ------------------------------------------------------------

TEST_F( KinematicsUtilsTest, DampedIdentityMatrix )
{
	MatXd identity = MatXd::Identity( 6, 6 );
	MatXd damped;
	double damping_factor = 0.1;

	Damped( identity, damping_factor, damped );

	// Damped matrix should be close to identity for small damping factor
	MatXd expected = identity.transpose() * ( identity.transpose() * identity + pow( damping_factor, 2 ) * MatXd::Identity( 6, 6 ) );
	EXPECT_TRUE( damped.isApprox( expected, 1e-6 ) ) << "Damped matrix should match expected values";
}

// ------------------------------------------------------------

TEST_F( KinematicsUtilsTest, PoseErrorIdentityPose )
{
	Mat4d target = Mat4d::Identity();
	Mat4d current = Mat4d::Identity();
	Vec6d pose_error;

	PoseError( target, current, pose_error );

	// Pose error should be zero for identical poses
	Vec6d expected_error = Vec6d::Zero();
	EXPECT_TRUE( pose_error.isApprox( expected_error, 1e-6 ) ) << "Pose error should be zero for identical poses";
}

// ------------------------------------------------------------

TEST_F( KinematicsUtilsTest, PoseErrorNonIdentityPose )
{
	Mat4d target = Mat4d::Identity();
	target( 0, 3 ) = 1.0; // Translation in x

	Mat4d current = Mat4d::Identity();
	Vec6d pose_error;

	PoseError( target, current, pose_error );

	// Expected pose error: translation error in x, zero rotation error
	Vec6d expected_error;
	expected_error << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0;
	EXPECT_TRUE( pose_error.isApprox( expected_error, 1e-6 ) ) << "Pose error should match expected values";
}

// ------------------------------------------------------------

TEST_F( KinematicsUtilsTest, POEWithSpan )
{
	const auto& joint_chain = Data::GetRevoluteOnlyRobotJointChain();
	Mat4d M = Data::GetRevoluteOnlyRobotHome();

	std::vector< double > thetas = { 1.5708, 0.7854, 0.3927 };
	std::span< const double > thetas_span( thetas );
	auto transform  = Data::GetRevoluteOnlyRobotTransform( thetas[0], thetas[1], thetas[2] );
	Mat4d poe;
	POE( joint_chain, M, thetas_span, poe );

	EXPECT_TRUE( poe.isApprox( transform ) ) <<
	    "POE=\n" << poe <<
	    "\nTransform=\n" << transform << std::endl;
}

// ------------------------------------------------------------

TEST_F( KinematicsUtilsTest, POEWithVecXd )
{
	const auto& joint_chain = Data::GetRevoluteOnlyRobotJointChain();
	Mat4d M = Data::GetRevoluteOnlyRobotHome();

	VecXd thetas( 3 );
	thetas << 1.5708, 0.7854, 0.3927;
	auto transform  = Data::GetRevoluteOnlyRobotTransform( thetas[0], thetas[1], thetas[2] );
	Mat4d poe;
	POE( joint_chain, M, thetas, poe );

	EXPECT_TRUE( poe.isApprox( transform ) );
}

// ------------------------------------------------------------

}
