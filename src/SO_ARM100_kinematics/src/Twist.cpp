#include "Global.hpp"

#include "Twist.hpp"

#include <gtest/gtest.h>
#include <Eigen/Dense>

namespace SOArm100::Kinematics::Test
{
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

TEST_F( TwistTest, DefaultConstructor )
{
	Twist twist;
	ASSERT_TRUE( static_cast< Vec6d >( twist ).isApprox( Vec6d::Zero() ) );
	ASSERT_TRUE( twist.GetAxis().isApprox( Vec3d::Zero() ) );
	ASSERT_TRUE( twist.GetLinear().isApprox( Vec3d::Zero() ) );
}

// ------------------------------------------------------------

TEST_F( TwistTest, ConstructorWithLinear )
{
	Vec3d linear( 1.0, 2.0, 3.0 );
	Twist twist( linear );

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

	Twist twist( axis, point_on_axis );

	Vec3d expected_linear = -axis.normalized().cross( point_on_axis );
	Vec3d expected_axis = axis.normalized();

	ASSERT_TRUE( twist.GetAxis().isApprox( expected_axis ) );
	ASSERT_TRUE( twist.GetLinear().isApprox( expected_linear ) );
}

// ------------------------------------------------------------

TEST_F( TwistTest, ExponentialMatrix )
{
	Vec3d axis( 0.0, 0.0, 1.0 );
	Vec3d point_on_axis( 0.5, 0.5, 1.5 );
	Twist twist( axis, point_on_axis );

	double theta = M_PI / 2.0; // 90 degrees
	Mat4d exponential = twist.ExponentialMatrix( theta );

	// Expected rotation matrix for 90 degrees around z-axis
	Mat3d expected_rotation;
	expected_rotation << 0.0, -1.0, 0.0,
	    1.0,  0.0, 0.0,
	    0.0,  0.0, 1.0;

	// Expected translation vector
	Vec3d expected_translation = -expected_rotation* twist.GetLinear() + twist.GetAxis() * theta * twist.GetLinear().dot( twist.GetAxis() );
	Mat3d rot = exponential.block< 3, 3 >( 0, 0 );
	Mat3d trans = exponential.block< 3, 1 >( 0, 3 );
	ASSERT_TRUE( rot.isApprox( expected_rotation, 1e-6 ) );
	ASSERT_TRUE( trans.isApprox( expected_translation, 1e-6 ) );
}

// ------------------------------------------------------------

} // namespace SOArm100::Kinematics::Test
