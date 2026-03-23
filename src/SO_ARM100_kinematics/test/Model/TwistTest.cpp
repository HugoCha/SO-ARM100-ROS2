#include "Global.hpp"

#include "Model/Twist.hpp"

#include <gtest/gtest.h>
#include <Eigen/Dense>

namespace SOArm100::Kinematics::Test
{

// ------------------------------------------------------------
// ------------------------------------------------------------

class TwistTest : public ::testing::Test
{
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

TEST_F( TwistTest, DefaultConstructor )
{
	Model::Twist twist;
	ASSERT_TRUE( static_cast< Vec6d >( twist ).isApprox( Vec6d::Zero() ) );
	ASSERT_TRUE( twist.GetAxis().isApprox( Vec3d::Zero() ) );
	ASSERT_TRUE( twist.GetLinear().isApprox( Vec3d::Zero() ) );
}

// ------------------------------------------------------------

TEST_F( TwistTest, ConstructorWithLinear )
{
	Vec3d linear( 1.0, 2.0, 3.0 );
	Model::Twist twist( linear );

	Vec6d expected_twist;
	expected_twist << 0.0, 0.0, 0.0, 1.0, 2.0, 3.0;
	expected_twist.tail< 3 >().normalize();

	ASSERT_TRUE( twist.GetLinear().isApprox( expected_twist.tail< 3 >() ) );
	ASSERT_TRUE( twist.GetAxis().isApprox( Vec3d::Zero() ) );
}

// ------------------------------------------------------------

TEST_F( TwistTest, ConstructorWithPointOnAxis )
{
	Vec3d axis( 0.0, 0.0, 1.0 );
	Vec3d point_on_axis( 0.5, 0.5, 1.5 );

	Model::Twist twist( axis, point_on_axis );

	Vec3d expected_linear = -axis.normalized().cross( point_on_axis );
	Vec3d expected_axis = axis.normalized();

	ASSERT_TRUE( twist.GetAxis().isApprox( expected_axis ) ) <<
	    "axis expected:\n" << expected_axis << "\nreal:\n" << twist.GetAxis();
	ASSERT_TRUE( twist.GetLinear().isApprox( expected_linear ) ) <<
	    "linear expected:\n" << expected_linear << "\nreal:\n" << twist.GetLinear();
}

// ------------------------------------------------------------

TEST_F( TwistTest, ExponentialMatrixWithFixed )
{
	Model::Twist twist;

	double theta = 1000;
	Mat4d exponential = twist.ExponentialMatrix( theta );

	// Expected rotation matrix for 90 degrees around z-axis
	Mat3d expected_rotation = Mat3d::Identity();

	// Expected translation vector
	Vec3d expected_translation = Vec3d::Zero();

	Mat3d rot = exponential.block< 3, 3 >( 0, 0 );
	Vec3d trans = exponential.block< 3, 1 >( 0, 3 );
	ASSERT_TRUE( rot.isApprox( expected_rotation, 1e-6 ) ) <<
	    "Rotation expected:\n" << expected_rotation << "\nreal\n" << rot;
	ASSERT_TRUE( trans.isApprox( expected_translation, 1e-6 ) ) <<
	    "Translation expected:\n" << expected_translation << "\nreal\n" << trans;
}

// ------------------------------------------------------------

TEST_F( TwistTest, ExponentialMatrixWithRevolute )
{
	Vec3d axis( 0.0, 0.0, 1.0 );
	Vec3d point_on_axis( 0.5, 0.5, 1.5 );
	Model::Twist twist( axis, point_on_axis );

	double theta = M_PI / 2.0; // 90 degrees
	Mat4d exponential = twist.ExponentialMatrix( theta );

	// Expected rotation matrix for 90 degrees around z-axis
	Mat3d expected_rotation;
	expected_rotation << 0.0, -1.0, 0.0,
	    1.0,  0.0, 0.0,
	    0.0,  0.0, 1.0;

	// Expected translation vector
	Vec3d expected_translation;
	expected_translation << 1.0, 0.0, 0.0;

	Mat3d rot = exponential.block< 3, 3 >( 0, 0 );
	Vec3d trans = exponential.block< 3, 1 >( 0, 3 );
	ASSERT_TRUE( rot.isApprox( expected_rotation, 1e-6 ) ) <<
	    "Rotation expected:\n" << expected_rotation << "\nreal\n" << rot;
	ASSERT_TRUE( trans.isApprox( expected_translation, 1e-6 ) ) <<
	    "Translation expected:\n" << expected_translation << "\nreal\n" << trans;
}

// ------------------------------------------------------------

TEST_F( TwistTest, ExponentialMatrixWithPrismatic )
{
	Vec3d axis( 0.0, 0.0, 1.0 );
	Model::Twist twist( axis );

	double theta = 0.1; // 0.1 m
	Mat4d exponential = twist.ExponentialMatrix( theta );

	// Expected rotation matrix for 90 degrees around z-axis
	Mat3d expected_rotation = Mat3d::Identity();

	// Expected translation vector
	Vec3d expected_translation;
	expected_translation << 0.0, 0.0, 0.1;

	Mat3d rot = exponential.block< 3, 3 >( 0, 0 );
	Vec3d trans = exponential.block< 3, 1 >( 0, 3 );
	ASSERT_TRUE( rot.isApprox( expected_rotation, 1e-6 ) ) <<
	    "Rotation expected:\n" << expected_rotation << "\nreal\n" << rot;
	ASSERT_TRUE( trans.isApprox( expected_translation, 1e-6 ) ) <<
	    "Translation expected:\n" << expected_translation << "\nreal\n" << trans;
}

// ------------------------------------------------------------

} // namespace SOArm100::Kinematics::Test
