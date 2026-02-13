#include "Twist.hpp"

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

	Twist twist( axis, point_on_axis, 0, M_PI );

	Vec3d expected_linear( 0.5, -0.5, 0.0 );
	ASSERT_TRUE( twist.GetAxis().isApprox( axis.normalized() ) );
	ASSERT_TRUE( twist.GetLinear().isApprox( expected_linear ) );
}

// ------------------------------------------------------------

} // namespace SOArm100::Kinematics::Test
