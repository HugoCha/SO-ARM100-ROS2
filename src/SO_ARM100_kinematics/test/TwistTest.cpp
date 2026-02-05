#include "Twist.hpp"
#include "Types.hpp"

#include <gtest/gtest.h>

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

TEST_F( TwistTest, ConstructorWithPointOnAxis )
{
	Vec3d axis( 0.0, 0.0, 1.0 );
	Vec3d point_on_axis( 0.5, 0.5, 1.5 );

	Twist twist( axis, point_on_axis );

	Vec3d expected_linear( 0.5, -0.5, 0.0 );
	ASSERT_TRUE( twist.GetAxis().isApprox( axis.normalized() ) );
	ASSERT_TRUE( twist.GetLinear().isApprox( expected_linear ) );
}

// ------------------------------------------------------------

TEST_F( TwistTest, ConstructorWithTransform )
{
	Vec3d axis( 0.0, 0.0, 1.0 );
	Mat4d transform = Mat4d::Identity();
	transform.block< 3, 1 >( 0, 3 ) = Vec3d { 0.5, 0.5, 0.5 };       // z = 0.5

	Twist twist( axis, transform );

	Vec3d expected_linear( 0.5, -0.5, 0.0 );
	ASSERT_TRUE( twist.GetAxis().isApprox( axis.normalized() ) );
	ASSERT_TRUE( twist.GetLinear().isApprox( expected_linear ) );
}

// ------------------------------------------------------------

} // namespace SOArm100::Kinematics::Test
