#include "Utils/MathUtils.hpp"

#include "Global.hpp"

#include <gtest/gtest.h>
#include <cmath>

namespace SOArm100::Kinematics::Test
{

// ============================================================
// Math Utils Test
// ============================================================

class MathUtilsTest : public ::testing::Test {};

// ============================================================
// ProjectPointOnPlane
// ============================================================

TEST_F( MathUtilsTest, ProjectPointOnPlane_PointAbovePlane_ProjectsOntoPlane )
{
	// XY plane (normal = Z), point at (1, 2, 5) → projection = (1, 2, 0)
	Vec3d result = ProjectPointOnPlane( Vec3d( 1, 2, 5 ), Vec3d( 0, 0, 0 ), Vec3d( 0, 0, 1 ) );
	EXPECT_TRUE( result.isApprox( Vec3d( 1, 2, 0 ) ) )
	    << "Result = " << result.transpose();
}

// ------------------------------------------------------------

TEST_F( MathUtilsTest, ProjectPointOnPlane_PointOnPlane_ReturnsPoint )
{
	// Point already on XY plane → displacement from plane_point is perpendicular to normal → zero
	Vec3d point_on_plane = Vec3d( 3, 4, 0 );
	Vec3d result = ProjectPointOnPlane( point_on_plane, Vec3d( 0, 0, 0 ), Vec3d( 0, 0, 1 ) );
	EXPECT_TRUE( result.isApprox( point_on_plane ) )
	    << "Point on plane should return point, got: " << result.transpose();
}

// ------------------------------------------------------------

TEST_F( MathUtilsTest, ProjectPointOnPlane_NonOriginPlanePoint )
{
	// Plane passing through (0, 0, 1) with normal Z; point at (2, 3, 4)
	// dir = (2, 3, 3); projection onto plane = (2, 3, 0)
	Vec3d result = ProjectPointOnPlane( Vec3d( 2, 3, 4 ), Vec3d( 0, 0, 1 ), Vec3d( 0, 0, 1 ) );
	EXPECT_TRUE( result.isApprox( Vec3d( 2, 3, 1 ) ) )
	    << "Result = " << result.transpose();
}

// ------------------------------------------------------------

TEST_F( MathUtilsTest, ProjectPointOnPlane_NonAxisAlignedNormal )
{
	// Plane with normal (1, 1, 0) / sqrt(2); point at (1, 1, 0)
	// dir = (1, 1, 0); dot(dir, n) = sqrt(2); projection = (1,1,0) - sqrt(2)*(1/sqrt2, 1/sqrt2, 0) = (0,0,0)
	Vec3d result = ProjectPointOnPlane( Vec3d( 1, 1, 0 ), Vec3d( 0, 0, 0 ), Vec3d( 1, 1, 0 ) );
	EXPECT_TRUE( result.isApprox( Vec3d::Zero(), 1e-9 ) )
	    << "Result = " << result.transpose();
}

// ------------------------------------------------------------

TEST_F( MathUtilsTest, ProjectPointOnPlane_UnnormalizedNormal_SameResultAsNormalized )
{
	Vec3d n_unit  = Vec3d( 0, 0, 1 );
	Vec3d n_long  = Vec3d( 0, 0, 5 );
	Vec3d point   = Vec3d( 1, 2, 3 );
	Vec3d plane_pt = Vec3d( 0, 0, 0 );

	Vec3d r1 = ProjectPointOnPlane( point, plane_pt, n_unit );
	Vec3d r2 = ProjectPointOnPlane( point, plane_pt, n_long );

	EXPECT_TRUE( r1.isApprox( r2, 1e-9 ) )
	    << "Unnormalized normal should give same result as normalized";
}

// ------------------------------------------------------------

TEST_F( MathUtilsTest, ProjectPointOnPlane_PointEqualsPlanePoint_ReturnsPoint )
{
	Vec3d pt = Vec3d( 1, 2, 3 );
	Vec3d result = ProjectPointOnPlane( pt, pt, Vec3d( 0, 0, 1 ) );
	EXPECT_TRUE( result.isApprox( pt ) )
	    << "Point == plane_point should return zero";
}

// ============================================================
// ProjectVectorOnPlane
// ============================================================

TEST_F( MathUtilsTest, ProjectVectorOnPlane_VectorInPlane_ReturnsSameVector )
{
	// Vector (1, 0, 0) is already in the XZ plane (normal = Y)
	Vec3d v      = Vec3d( 1, 0, 0 );
	Vec3d normal = Vec3d( 0, 1, 0 );
	Vec3d result = ProjectVectorOnPlane( v, normal );
	EXPECT_TRUE( result.isApprox( v ) )
	    << "Vector in plane should be unchanged, got: " << result.transpose();
}

// ------------------------------------------------------------

TEST_F( MathUtilsTest, ProjectVectorOnPlane_VectorAlongNormal_ReturnsZero )
{
	Vec3d v      = Vec3d( 0, 0, 5 );
	Vec3d normal = Vec3d( 0, 0, 1 );
	Vec3d result = ProjectVectorOnPlane( v, normal );
	EXPECT_TRUE( result.isApprox( Vec3d::Zero() ) )
	    << "Vector parallel to normal should project to zero";
}

// ------------------------------------------------------------

TEST_F( MathUtilsTest, ProjectVectorOnPlane_GeneralVector_RemovesNormalComponent )
{
	// v = (1, 0, 1), normal = Z → removes Z component → (1, 0, 0)
	Vec3d result = ProjectVectorOnPlane( Vec3d( 1, 0, 1 ), Vec3d( 0, 0, 1 ) );
	EXPECT_TRUE( result.isApprox( Vec3d( 1, 0, 0 ) ) )
	    << "Result = " << result.transpose();
}

// ------------------------------------------------------------

TEST_F( MathUtilsTest, ProjectVectorOnPlane_UnnormalizedNormal_SameResultAsNormalized )
{
	Vec3d v      = Vec3d( 3, 4, 5 );
	Vec3d r1     = ProjectVectorOnPlane( v, Vec3d( 0, 0, 1 ) );
	Vec3d r2     = ProjectVectorOnPlane( v, Vec3d( 0, 0, 10 ) );
	EXPECT_TRUE( r1.isApprox( r2, 1e-9 ) );
}

// ------------------------------------------------------------

TEST_F( MathUtilsTest, ProjectVectorOnPlane_ZeroVector_ReturnsZero )
{
	Vec3d result = ProjectVectorOnPlane( Vec3d::Zero(), Vec3d( 0, 0, 1 ) );
	EXPECT_TRUE( result.isApprox( Vec3d::Zero() ) );
}

// ------------------------------------------------------------

TEST_F( MathUtilsTest, ProjectVectorOnPlane_ResultIsOrthogonalToNormal )
{
	Vec3d v      = Vec3d( 1, 2, 3 );
	Vec3d normal = Vec3d( 1, 1, 1 ).normalized();
	Vec3d result = ProjectVectorOnPlane( v, normal );
	EXPECT_NEAR( result.dot( normal ), 0.0, 1e-9 )
	    << "Projected vector should be orthogonal to the plane normal";
}

// ============================================================
// ProjectPointOnAxis
// ============================================================

TEST_F( MathUtilsTest, ProjectPointOnAxis_PointOnAxis_ReturnsPointMinusOrigin )
{
	// Point (0, 0, 3), origin (0, 0, 0), axis Z → projection = (0, 0, 3)
	Vec3d result = ProjectPointOnAxis( Vec3d( 0, 0, 3 ), Vec3d( 0, 0, 0 ), Vec3d( 0, 0, 1 ) );
	EXPECT_TRUE( result.isApprox( Vec3d( 0, 0, 3 ) ) )
	    << "Result = " << result.transpose();
}

// ------------------------------------------------------------

TEST_F( MathUtilsTest, ProjectPointOnAxis_PointPerpendicularToAxis_ReturnsOrigin )
{
	// Point (1, 0, 0), axis Z → no component along Z
	Vec3d origin = Vec3d( 2, 0, 1 );
	Vec3d point  = Vec3d::UnitX();
	Vec3d expected = Vec3d( origin.x(), origin.y(), point.z() );
	Vec3d axis = Vec3d::UnitZ();
	Vec3d result = ProjectPointOnAxis( point, origin, axis );
	EXPECT_TRUE( result.isApprox( expected ) )
	    << "Expected = " << expected.transpose() << std::endl
	    << "Result   = " << result.transpose() << std::endl;
}

// ------------------------------------------------------------

TEST_F( MathUtilsTest, ProjectPointOnAxis_NonOriginAxisOrigin )
{
	// Point (3, 0, 2), origin (3, 0, 0), axis Z → dir = (0, 0, 2) → projection = (0, 0, 2)
	Vec3d origin = Vec3d( 3, 4, 0 );
	Vec3d point  = Vec3d( 2, 1, 2 );
	Vec3d expected = Vec3d( origin.x(), origin.y(), point.z() );
	Vec3d axis = Vec3d::UnitZ();
	Vec3d result = ProjectPointOnAxis( point, origin, axis );
	EXPECT_TRUE( result.isApprox( expected ) )
	    << "Result = " << result.transpose();
}

// ------------------------------------------------------------

TEST_F( MathUtilsTest, ProjectPointOnAxis_GeneralPoint_ExtractsAxisComponent )
{
	// Point (1, 2, 3), origin (0,0,0), axis Z → (0, 0, 3)
	Vec3d result = ProjectPointOnAxis( Vec3d( 1, 2, 3 ), Vec3d( 0, 0, 0 ), Vec3d( 0, 0, 1 ) );
	EXPECT_TRUE( result.isApprox( Vec3d( 0, 0, 3 ) ) );
}

// ------------------------------------------------------------

TEST_F( MathUtilsTest, ProjectPointOnAxis_UnnormalizedAxis_SameResultAsNormalized )
{
	Vec3d pt     = Vec3d( 1, 2, 3 );
	Vec3d origin = Vec3d( 0, 0, 0 );
	Vec3d r1 = ProjectPointOnAxis( pt, origin, Vec3d( 0, 0, 1 ) );
	Vec3d r2 = ProjectPointOnAxis( pt, origin, Vec3d( 0, 0, 7 ) );
	EXPECT_TRUE( r1.isApprox( r2, 1e-9 ) );
}

// ------------------------------------------------------------

TEST_F( MathUtilsTest, ProjectPointOnAxis_ResultIsParallelToAxis )
{
	Vec3d axis   = Vec3d( 1, 1, 0 ).normalized();
	Vec3d result = ProjectPointOnAxis( Vec3d( 3, 1, 2 ), Vec3d( 0, 0, 0 ), axis );
	// Cross product of result and axis should be zero (parallel)
	EXPECT_TRUE( result.cross( axis ).norm() < 1e-9 )
	    << "Projected vector should be parallel to axis";
}

// ============================================================
// Angle
// ============================================================

TEST_F( MathUtilsTest, Angle_SameVector_ReturnsZero )
{
	Vec3d v = Vec3d( 1, 2, 3 );
	EXPECT_NEAR( Angle( v, v ), 0.0, 1e-9 );
}

// ------------------------------------------------------------

TEST_F( MathUtilsTest, Angle_OppositeVectors_ReturnsPi )
{
	EXPECT_NEAR( Angle( Vec3d( 1, 0, 0 ), Vec3d( -1, 0, 0 ) ), M_PI, 1e-9 );
}

// ------------------------------------------------------------

TEST_F( MathUtilsTest, Angle_OrthogonalVectors_ReturnsHalfPi )
{
	EXPECT_NEAR( Angle( Vec3d( 1, 0, 0 ), Vec3d( 0, 1, 0 ) ), M_PI / 2.0, 1e-9 );
}

// ------------------------------------------------------------

TEST_F( MathUtilsTest, Angle_KnownAngle_45Degrees )
{
	Vec3d v1( 1, 0, 0 );
	Vec3d v2( 1, 1, 0 );
	EXPECT_NEAR( Angle( v1, v2 ), M_PI / 4.0, 1e-9 );
}

// ------------------------------------------------------------

TEST_F( MathUtilsTest, Angle_AlwaysNonNegative )
{
	// Angle is always in [0, pi] — never negative
	EXPECT_GE( Angle( Vec3d( 1, 0, 0 ), Vec3d( 0, -1, 0 ) ), 0.0 );
	EXPECT_GE( Angle( Vec3d( 0, 1, 0 ), Vec3d( -1, 0, 0 ) ), 0.0 );
}

// ------------------------------------------------------------

TEST_F( MathUtilsTest, Angle_Symmetric_V1V2EqualsV2V1 )
{
	Vec3d v1( 1, 2, 3 );
	Vec3d v2( 4, 5, 6 );
	EXPECT_NEAR( Angle( v1, v2 ), Angle( v2, v1 ), 1e-9 );
}

// ------------------------------------------------------------

TEST_F( MathUtilsTest, Angle_UnnormalizedVectors_SameResultAsNormalized )
{
	Vec3d v1( 3, 0, 0 );
	Vec3d v2( 0, 7, 0 );
	EXPECT_NEAR( Angle( v1, v2 ), M_PI / 2.0, 1e-9 );
}

// ============================================================
// SignedAngle
// ============================================================

TEST_F( MathUtilsTest, SignedAngle_SameVector_ReturnsZero )
{
	Vec3d v = Vec3d( 1, 0, 0 );
	EXPECT_NEAR( SignedAngle( v, v, Vec3d( 0, 0, 1 ) ), 0.0, 1e-9 );
}

// ------------------------------------------------------------

TEST_F( MathUtilsTest, SignedAngle_PositiveRotation_CCW_AboutZ )
{
	// +X rotated 90° CCW about Z → +Y; expected +pi/2
	EXPECT_NEAR(
		SignedAngle( Vec3d( 1, 0, 0 ), Vec3d( 0, 1, 0 ), Vec3d( 0, 0, 1 ) ),
		M_PI / 2.0, 1e-9 );
}

// ------------------------------------------------------------

TEST_F( MathUtilsTest, SignedAngle_NegativeRotation_CW_AboutZ )
{
	// +X rotated 90° CW about Z → -Y; expected -pi/2
	EXPECT_NEAR(
		SignedAngle( Vec3d( 1, 0, 0 ), Vec3d( 0, -1, 0 ), Vec3d( 0, 0, 1 ) ),
		-M_PI / 2.0, 1e-9 );
}

// ------------------------------------------------------------

TEST_F( MathUtilsTest, SignedAngle_AntiSymmetric_SwappingVectorsNegatesSign )
{
	Vec3d v1( 1, 0, 0 );
	Vec3d v2( 0, 1, 0 );
	Vec3d n( 0, 0, 1 );
	EXPECT_NEAR(
		SignedAngle( v1, v2, n ),
		-SignedAngle( v2, v1, n ),
		1e-9 );
}

// ------------------------------------------------------------

TEST_F( MathUtilsTest, SignedAngle_FlippingNormal_NegatesSign )
{
	Vec3d v1( 1, 0, 0 );
	Vec3d v2( 0, 1, 0 );
	EXPECT_NEAR(
		SignedAngle( v1, v2, Vec3d( 0, 0,  1 ) ),
		-SignedAngle( v1, v2, Vec3d( 0, 0, -1 ) ),
		1e-9 );
}

// ------------------------------------------------------------

TEST_F( MathUtilsTest, SignedAngle_ZeroV1_ReturnsZero )
{
	EXPECT_NEAR(
		SignedAngle( Vec3d::Zero(), Vec3d( 1, 0, 0 ), Vec3d( 0, 0, 1 ) ),
		0.0, 1e-9 );
}

// ------------------------------------------------------------

TEST_F( MathUtilsTest, SignedAngle_ZeroV2_ReturnsZero )
{
	EXPECT_NEAR(
		SignedAngle( Vec3d( 1, 0, 0 ), Vec3d::Zero(), Vec3d( 0, 0, 1 ) ),
		0.0, 1e-9 );
}

// ------------------------------------------------------------

TEST_F( MathUtilsTest, SignedAngle_ZeroNormal_ReturnsZero )
{
	EXPECT_NEAR(
		SignedAngle( Vec3d( 1, 0, 0 ), Vec3d( 0, 1, 0 ), Vec3d::Zero() ),
		0.0, 1e-9 );
}

// ------------------------------------------------------------

TEST_F( MathUtilsTest, SignedAngle_UnnormalizedInputs_SameResultAsNormalized )
{
	Vec3d v1_long = Vec3d( 5, 0, 0 );
	Vec3d v2_long = Vec3d( 0, 3, 0 );
	Vec3d n_long  = Vec3d( 0, 0, 7 );

	double r1 = SignedAngle( v1_long, v2_long, n_long );
	double r2 = SignedAngle( Vec3d( 1, 0, 0 ), Vec3d( 0, 1, 0 ), Vec3d( 0, 0, 1 ) );

	EXPECT_NEAR( r1, r2, 1e-9 )
	    << "Unnormalized inputs should give same signed angle as normalized";
}

// ------------------------------------------------------------

TEST_F( MathUtilsTest, SignedAngle_180Degrees_ReturnsPI )
{
	// +X to -X, viewed from +Z
	double angle = SignedAngle( Vec3d( 1, 0, 0 ), Vec3d( -1, 0, 0 ), Vec3d( 0, 0, 1 ) );
	EXPECT_NEAR( std::abs( angle ), M_PI, 1e-9 );
}

// ------------------------------------------------------------

TEST_F( MathUtilsTest, SignedAngle_SmallAngle_AboutY )
{
	double theta = 0.1;
	Vec3d v1( 1, 0, 0 );
	Vec3d v2( std::cos( theta ), 0, -std::sin( theta ) );  // rotation about +Y
	EXPECT_NEAR( SignedAngle( v1, v2, Vec3d( 0, 1, 0 ) ), theta, 1e-9 );
}

// ------------------------------------------------------------

TEST_F( MathUtilsTest, SignedAngle_ResultInRange_MinusPiToPi )
{
	for ( int deg = -170; deg <= 170; deg += 10 )
	{
		double rad  = deg * M_PI / 180.0;
		Vec3d v1( 1, 0, 0 );
		Vec3d v2( std::cos( rad ), std::sin( rad ), 0 );
		double result = SignedAngle( v1, v2, Vec3d( 0, 0, 1 ) );
		EXPECT_GE( result, -M_PI - 1e-9 ) << "Angle out of range for deg=" << deg;
		EXPECT_LE( result,  M_PI + 1e-9 ) << "Angle out of range for deg=" << deg;
	}
}

// ------------------------------------------------------------

}