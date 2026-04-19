#include "Global.hpp"

#include "Model/Joint/JointChain.hpp"
#include "Model/Joint/Limits.hpp"
#include "Model/Joint/Twist.hpp"
#include "RobotModelTestData.hpp"
#include "Utils/Converter.hpp"
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
	transform.block< 3, 3 >( 0, 0 ) = AngleAxis( M_PI / 2, Eigen::Vector3d::UnitZ() ).toRotationMatrix();
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
	Model::JointChain joint_chain( 1 );
	Model::Twist twist( Vec3d( 0, 0, 1 ), Vec3d( 0, 0, 0 ) );
	Model::Link link( Mat4d::Identity(), 0 );
	Model::Limits limits( -M_PI, M_PI );
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
	Model::JointChain joint_chain( 2 );

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

TEST_F( KinematicsUtilsTest, SpaceJacobianZYZRevoluteRobotTwist )
{
	const auto& joint_chain = Data::GetZYZRevoluteRobotJointChain();
	VecXd joint_angles( 3 );
	joint_angles << 1.5708, 0.7854, 0.3927;

	MatXd geom_jacobian = Data::GetZYZRevoluteRobotJacobian(
		joint_angles[0],
		joint_angles[1],
		joint_angles[2] );
	Mat4d T0e = Data::GetZYZRevoluteRobotTransform( joint_angles[0], joint_angles[1], joint_angles[2] );
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
	const auto& joint_chain = Data::GetZYZRevoluteRobotJointChain();
	Mat4d M = Data::GetZYZRevoluteRobotHome();

	std::vector< double > thetas = { 1.5708, 0.7854, 0.3927 };
	std::span< const double > thetas_span( thetas );
	auto transform  = Data::GetZYZRevoluteRobotTransform( thetas[0], thetas[1], thetas[2] );
	Mat4d poe;
	POE( joint_chain, M, thetas_span, poe );

	EXPECT_TRUE( poe.isApprox( transform ) ) <<
	    "POE=\n" << poe <<
	    "\nTransform=\n" << transform << std::endl;
}

// ------------------------------------------------------------

TEST_F( KinematicsUtilsTest, POEWithVecXd )
{
	const auto& joint_chain = Data::GetZYZRevoluteRobotJointChain();
	Mat4d M = Data::GetZYZRevoluteRobotHome();

	VecXd thetas( 3 );
	thetas << 1.5708, 0.7854, 0.3927;
	auto transform  = Data::GetZYZRevoluteRobotTransform( thetas[0], thetas[1], thetas[2] );
	Mat4d poe;
	POE( joint_chain, M, thetas, poe );

	EXPECT_TRUE( poe.isApprox( transform ) );
}

// ------------------------------------------------------------

// Build a revolute joint whose axis passes through `point_on_axis` with direction `axis`.
// Twist(axis, point_on_axis) internally computes linear = -(axis_normalized × point_on_axis).
static Model::JointConstPtr MakeRevoluteJoint(
	const Vec3d& axis,
	const Vec3d& point_on_axis = Vec3d::Zero() )
{
	return std::make_shared< const Model::Joint >(
		Model::Twist( axis, point_on_axis ),
		Model::Link( ToTransformMatrix( point_on_axis ), 0 ),
		Model::Limits( -M_PI, M_PI )
		);
}

static std::span< const Model::JointConstPtr > ToSpan(
	const std::vector< Model::JointConstPtr >& v )
{
	return { v };
}

// ============================================================
// ComputeIntersection
// ============================================================

class ComputeIntersectionTest : public ::testing::Test {};

// ------------------------------------------------------------
// Concurrent axes — unique intersection exists
// ------------------------------------------------------------

TEST_F( ComputeIntersectionTest, ThreeConcurrentAxes_AtOrigin )
{
	// Three orthogonal axes all passing through the origin
	std::vector< Model::JointConstPtr > joints{
		MakeRevoluteJoint( Vec3d( 1, 0, 0 ), Vec3d::Zero() ),
		MakeRevoluteJoint( Vec3d( 0, 1, 0 ), Vec3d::Zero() ),
		MakeRevoluteJoint( Vec3d( 0, 0, 1 ), Vec3d::Zero() ),
	};

	auto result = ComputeIntersection( ToSpan( joints ) );

	ASSERT_TRUE( result.has_value() ) << "Three concurrent axes at origin should have an intersection";
	EXPECT_TRUE( result->isApprox( Vec3d::Zero(), 1e-9 ) )
	    << "Intersection should be at origin, got " << result->transpose();
}

TEST_F( ComputeIntersectionTest, ThreeConcurrentAxes_AtArbitraryPoint )
{
	// Three orthogonal axes all passing through (1, 2, 3)
	Vec3d point( 1, 2, 3 );

	std::vector< Model::JointConstPtr > joints{
		MakeRevoluteJoint( Vec3d( 1, 0, 0 ), point ),
		MakeRevoluteJoint( Vec3d( 0, 1, 0 ), point ),
		MakeRevoluteJoint( Vec3d( 0, 0, 1 ), point ),
	};

	auto result = ComputeIntersection( ToSpan( joints ) );

	ASSERT_TRUE( result.has_value() ) << "Three concurrent axes should have an intersection";
	EXPECT_TRUE( result->isApprox( point, 1e-9 ) )
	    << "Expected " << point.transpose() << ", got " << result->transpose();
}

TEST_F( ComputeIntersectionTest, ThreeConcurrentAxes_AtNegativePoint )
{
	Vec3d point( -2, -3, -4 );

	std::vector< Model::JointConstPtr > joints{
		MakeRevoluteJoint( Vec3d( 1, 0, 0 ), point ),
		MakeRevoluteJoint( Vec3d( 0, 1, 0 ), point ),
		MakeRevoluteJoint( Vec3d( 0, 0, 1 ), point ),
	};

	auto result = ComputeIntersection( ToSpan( joints ) );

	ASSERT_TRUE( result.has_value() );
	EXPECT_TRUE( result->isApprox( point, 1e-9 ) )
	    << "Got " << result->transpose();
}

TEST_F( ComputeIntersectionTest, ThreeConcurrentAxes_NonOrthogonalAxes )
{
	// Non-orthogonal but linearly independent axes, all through (1, 1, 1)
	Vec3d point( 1, 1, 1 );

	std::vector< Model::JointConstPtr > joints{
		MakeRevoluteJoint( Vec3d( 1, 0, 0 ), point ),
		MakeRevoluteJoint( Vec3d( 1, 1, 0 ).normalized(), point ),
		MakeRevoluteJoint( Vec3d( 1, 1, 1 ).normalized(), point ),
	};

	auto result = ComputeIntersection( ToSpan( joints ) );

	ASSERT_TRUE( result.has_value() );
	EXPECT_TRUE( result->isApprox( point, 1e-9 ) )
	    << "Got " << result->transpose();
}

TEST_F( ComputeIntersectionTest, ThreeConcurrentAxes_DifferentPointsOnAxis_ReturnsIntersection )
{
	// Intersection at (2, -1, 4), each axis defined from a different point along its own line:
	//   - X-axis: defined from ( 5, -1,  4) — 3 units along X from intersection
	//   - Y-axis: defined from ( 2,  2,  4) — 3 units along Y from intersection
	//   - Z-axis: defined from ( 2, -1, -1) — 5 units along Z from intersection
	Vec3d intersection( 2, -1, 4 );

	std::vector< Model::JointConstPtr > joints{
		MakeRevoluteJoint( Vec3d( 1, 0, 0 ), Vec3d(  5, -1,  4 ) ),
		MakeRevoluteJoint( Vec3d( 0, 1, 0 ), Vec3d(  2,  2,  4 ) ),
		MakeRevoluteJoint( Vec3d( 0, 0, 1 ), Vec3d(  2, -1, -1 ) ),
	};

	auto result = ComputeIntersection( ToSpan( joints ) );

	ASSERT_TRUE( result.has_value() )
	    << "Three lines with different definition points but same intersection should succeed";
	EXPECT_TRUE( result->isApprox( intersection, 1e-9 ) )
	    << "Expected " << intersection.transpose() << ", got " << result->transpose();
}

TEST_F( ComputeIntersectionTest, TwoConcurrentAxes_DifferentPointsOnAxis_ReturnsIntersection )
{
	// Intersection at (1, 2, 3), but each axis is defined from a different point along its line:
	//   - X-axis line: direction (1,0,0), defined from point (3, 2, 3)  — 2 units away along X
	//   - Y-axis line: direction (0,1,0), defined from point (1, 5, 3)  — 3 units away along Y
	// Both lines still pass through (1, 2, 3).
	Vec3d intersection( 1, 2, 3 );

	// std::vector< Model::JointConstPtr > joints{
	//     MakeRevoluteJoint( Vec3d( 1, 0, 0 ), Vec3d( 3, 2, 3 ) ),
	//     MakeRevoluteJoint( Vec3d( 0, 1, 0 ), Vec3d( 1, 5, 3 ) ),
	// };

	std::vector< Model::JointConstPtr > joints{
		MakeRevoluteJoint( Vec3d( 1, 0, 0 ), Vec3d( 0, 0, 1.5 ) ),
		MakeRevoluteJoint( Vec3d( 0, 0, 1 ), Vec3d( 0.1, 0, 1.6 ) ),
	};
	auto result = ComputeIntersection( ToSpan( joints ) );
	Vec3d expected_center = Vec3d( 0.1, 0, 1.5 );

	ASSERT_TRUE( result.has_value() )
	    << "Two lines with different definition points but same intersection should succeed";
	ASSERT_TRUE( expected_center.isApprox( *result ) )
	    << "Expected = " << expected_center.transpose() << std::endl
	    << "Result   = " << result->transpose() << std::endl;
}

TEST_F( ComputeIntersectionTest, TwoConcurrentAxes_ReturnsIntersection )
{
	// Two non-parallel, intersecting axes through (0, 0, 1)
	Vec3d point( 0, 0, 1 );

	std::vector< Model::JointConstPtr > joints{
		MakeRevoluteJoint( Vec3d( 1, 0, 0 ), point ),
		MakeRevoluteJoint( Vec3d( 0, 1, 0 ), point ),
	};

	auto result = ComputeIntersection( ToSpan( joints ) );

	ASSERT_TRUE( result.has_value() );
	// The Z component is undetermined with only 2 axes; X and Y must match
	EXPECT_NEAR( result->x(), point.x(), 1e-9 );
	EXPECT_NEAR( result->y(), point.y(), 1e-9 );
}

TEST_F( ComputeIntersectionTest, UnnormalizedAxis_SameResultAsNormalized )
{
	// The Twist constructor normalises the axis internally,
	// so a scaled axis should yield the same intersection.
	Vec3d point( 1, 2, 3 );

	std::vector< Model::JointConstPtr > joints_unit{
		MakeRevoluteJoint( Vec3d( 1, 0, 0 ), point ),
		MakeRevoluteJoint( Vec3d( 0, 1, 0 ), point ),
		MakeRevoluteJoint( Vec3d( 0, 0, 1 ), point ),
	};

	std::vector< Model::JointConstPtr > joints_scaled{
		MakeRevoluteJoint( Vec3d( 5, 0, 0 ), point ),
		MakeRevoluteJoint( Vec3d( 0, 3, 0 ), point ),
		MakeRevoluteJoint( Vec3d( 0, 0, 7 ), point ),
	};

	auto r1 = ComputeIntersection( ToSpan( joints_unit ) );
	auto r2 = ComputeIntersection( ToSpan( joints_scaled ) );

	ASSERT_TRUE( r1.has_value() );
	ASSERT_TRUE( r2.has_value() );
	EXPECT_TRUE( r1->isApprox( *r2, 1e-9 ) );
}

// ------------------------------------------------------------
// Non-concurrent axes — no exact intersection
// ------------------------------------------------------------

TEST_F( ComputeIntersectionTest, TwoParallelAxes_ReturnsNullopt )
{
	// Two parallel axes along Z, offset from each other in X — they never meet
	std::vector< Model::JointConstPtr > joints{
		MakeRevoluteJoint( Vec3d( 0, 0, 1 ), Vec3d( 0, 0, 0 ) ),
		MakeRevoluteJoint( Vec3d( 0, 0, 1 ), Vec3d( 1, 0, 0 ) ),
	};

	auto result = ComputeIntersection( ToSpan( joints ) );

	EXPECT_FALSE( result.has_value() )
	    << "Parallel non-coincident axes should return nullopt";
}

TEST_F( ComputeIntersectionTest, ThreeSkewAxes_ReturnsNullopt )
{
	// Three skew lines: each axis is offset so they do not share a common point
	std::vector< Model::JointConstPtr > joints{
		MakeRevoluteJoint( Vec3d( 1, 0, 0 ), Vec3d( 0, 1, 0 ) ),  // X-axis shifted +1 in Y
		MakeRevoluteJoint( Vec3d( 0, 1, 0 ), Vec3d( 0, 0, 1 ) ),  // Y-axis shifted +1 in Z
		MakeRevoluteJoint( Vec3d( 0, 0, 1 ), Vec3d( 1, 0, 0 ) ),  // Z-axis shifted +1 in X
	};

	auto result = ComputeIntersection( ToSpan( joints ) );

	EXPECT_FALSE( result.has_value() )
	    << "Three skew axes should return nullopt" << std::endl
	    << "Result = " << result->transpose() << std::endl;
}

TEST_F( ComputeIntersectionTest, ThreeConcurrentAxes_TwoParallel_ReturnsNullopt )
{
	// Two axes are parallel → the system is inconsistent unless their offsets also match
	Vec3d point( 1, 2, 3 );

	std::vector< Model::JointConstPtr > joints{
		MakeRevoluteJoint( Vec3d( 0, 0, 1 ), point ),              // Z through point
		MakeRevoluteJoint( Vec3d( 0, 0, 1 ), Vec3d( 2, 0, 0 ) ),  // Z offset — skew
		MakeRevoluteJoint( Vec3d( 1, 0, 0 ), point ),
	};

	auto result = ComputeIntersection( ToSpan( joints ) );

	EXPECT_FALSE( result.has_value() );
}

// ============================================================
// AxesIndependent
// ============================================================

class AxesIndependentTest : public ::testing::Test {};

// ------------------------------------------------------------
// Single joint
// ------------------------------------------------------------

TEST_F( AxesIndependentTest, SingleJoint_NonZeroAxis_ReturnsTrue )
{
	std::vector< Model::JointConstPtr > joints{
		MakeRevoluteJoint( Vec3d( 0, 0, 1 ) ),
	};
	EXPECT_TRUE( AxesIndependent( ToSpan( joints ) ) );
}

TEST_F( AxesIndependentTest, SingleJoint_ZeroAxis_ReturnsFalse )
{
	// A zero-length axis: use the prismatic Twist constructor (linear only)
	// and build a joint manually so the axis stays zero.
	auto joint = std::make_shared< const Model::Joint >(
		Model::Twist( Vec3d::Zero() ),   // prismatic with zero linear → axis = (0,0,0)
		Model::Link(),
		Model::Limits( -M_PI, M_PI )
		);

	std::vector< Model::JointConstPtr > joints{ joint };
	EXPECT_FALSE( AxesIndependent( ToSpan( joints ) ) );
}

// ------------------------------------------------------------
// Two joints
// ------------------------------------------------------------

TEST_F( AxesIndependentTest, TwoJoints_OrthogonalAxes_ReturnsTrue )
{
	std::vector< Model::JointConstPtr > joints{
		MakeRevoluteJoint( Vec3d( 1, 0, 0 ) ),
		MakeRevoluteJoint( Vec3d( 0, 1, 0 ) ),
	};
	EXPECT_TRUE( AxesIndependent( ToSpan( joints ) ) );
}

TEST_F( AxesIndependentTest, TwoJoints_ParallelAxes_ReturnsFalse )
{
	std::vector< Model::JointConstPtr > joints{
		MakeRevoluteJoint( Vec3d( 0, 0, 1 ) ),
		MakeRevoluteJoint( Vec3d( 0, 0, 1 ) ),
	};
	EXPECT_FALSE( AxesIndependent( ToSpan( joints ) ) );
}

TEST_F( AxesIndependentTest, TwoJoints_AntiParallelAxes_ReturnsFalse )
{
	// w2 = -w1 → cross product is zero
	std::vector< Model::JointConstPtr > joints{
		MakeRevoluteJoint( Vec3d( 0, 0,  1 ) ),
		MakeRevoluteJoint( Vec3d( 0, 0, -1 ) ),
	};
	EXPECT_FALSE( AxesIndependent( ToSpan( joints ) ) );
}

TEST_F( AxesIndependentTest, TwoJoints_ScaledParallelAxes_ReturnsFalse )
{
	// Twist normalises the axis, so (0,0,2) and (0,0,1) are the same direction
	std::vector< Model::JointConstPtr > joints{
		MakeRevoluteJoint( Vec3d( 0, 0, 1 ) ),
		MakeRevoluteJoint( Vec3d( 0, 0, 2 ) ),
	};
	EXPECT_FALSE( AxesIndependent( ToSpan( joints ) ) );
}

TEST_F( AxesIndependentTest, TwoJoints_NonOrthogonalButIndependent_ReturnsTrue )
{
	// (1,0,0) and (1,1,0) — not orthogonal, but cross product is non-zero
	std::vector< Model::JointConstPtr > joints{
		MakeRevoluteJoint( Vec3d( 1, 0, 0 ) ),
		MakeRevoluteJoint( Vec3d( 1, 1, 0 ) ),
	};
	EXPECT_TRUE( AxesIndependent( ToSpan( joints ) ) );
}

// ------------------------------------------------------------
// Three joints
// ------------------------------------------------------------

TEST_F( AxesIndependentTest, ThreeJoints_OrthogonalAxes_ReturnsTrue )
{
	std::vector< Model::JointConstPtr > joints{
		MakeRevoluteJoint( Vec3d( 1, 0, 0 ) ),
		MakeRevoluteJoint( Vec3d( 0, 1, 0 ) ),
		MakeRevoluteJoint( Vec3d( 0, 0, 1 ) ),
	};
	EXPECT_TRUE( AxesIndependent( ToSpan( joints ) ) );
}

TEST_F( AxesIndependentTest, ThreeJoints_ThirdAxisInSpanOfFirstTwo_ReturnsFalse )
{
	// w3 = w1 + w2 → determinant is zero
	std::vector< Model::JointConstPtr > joints{
		MakeRevoluteJoint( Vec3d( 1, 0, 0 ) ),
		MakeRevoluteJoint( Vec3d( 0, 1, 0 ) ),
		MakeRevoluteJoint( Vec3d( 1, 1, 0 ).normalized() ),
	};
	EXPECT_FALSE( AxesIndependent( ToSpan( joints ) ) );
}

TEST_F( AxesIndependentTest, ThreeJoints_AllParallel_ReturnsFalse )
{
	std::vector< Model::JointConstPtr > joints{
		MakeRevoluteJoint( Vec3d( 0, 0, 1 ) ),
		MakeRevoluteJoint( Vec3d( 0, 0, 1 ) ),
		MakeRevoluteJoint( Vec3d( 0, 0, 1 ) ),
	};
	EXPECT_FALSE( AxesIndependent( ToSpan( joints ) ) );
}

TEST_F( AxesIndependentTest, ThreeJoints_TwoParallelOneIndependent_ReturnsFalse )
{
	// Two parallel Z axes → determinant is zero regardless of the third
	std::vector< Model::JointConstPtr > joints{
		MakeRevoluteJoint( Vec3d( 0, 0, 1 ) ),
		MakeRevoluteJoint( Vec3d( 0, 0, 1 ) ),
		MakeRevoluteJoint( Vec3d( 1, 0, 0 ) ),
	};
	EXPECT_FALSE( AxesIndependent( ToSpan( joints ) ) );
}

TEST_F( AxesIndependentTest, ThreeJoints_NonOrthogonalButFullRank_ReturnsTrue )
{
	std::vector< Model::JointConstPtr > joints{
		MakeRevoluteJoint( Vec3d( 1, 0, 0 ) ),
		MakeRevoluteJoint( Vec3d( 1, 1, 0 ).normalized() ),
		MakeRevoluteJoint( Vec3d( 1, 1, 1 ).normalized() ),
	};
	EXPECT_TRUE( AxesIndependent( ToSpan( joints ) ) );
}

// ------------------------------------------------------------
// More than three joints — always false
// ------------------------------------------------------------

TEST_F( AxesIndependentTest, FourJoints_OrthogonalAxes_ReturnsFalse )
{
	// AxesIndependent hard-returns false for k > 3
	std::vector< Model::JointConstPtr > joints{
		MakeRevoluteJoint( Vec3d( 1, 0, 0 ) ),
		MakeRevoluteJoint( Vec3d( 0, 1, 0 ) ),
		MakeRevoluteJoint( Vec3d( 0, 0, 1 ) ),
		MakeRevoluteJoint( Vec3d( 1, 1, 0 ).normalized() ),
	};
	EXPECT_FALSE( AxesIndependent( ToSpan( joints ) ) )
	    << "More than 3 joints should always return false";
}

TEST_F( AxesIndependentTest, FiveJoints_ReturnsFalse )
{
	std::vector< Model::JointConstPtr > joints{
		MakeRevoluteJoint( Vec3d( 1, 0, 0 ) ),
		MakeRevoluteJoint( Vec3d( 0, 1, 0 ) ),
		MakeRevoluteJoint( Vec3d( 0, 0, 1 ) ),
		MakeRevoluteJoint( Vec3d( 1, 1, 0 ).normalized() ),
		MakeRevoluteJoint( Vec3d( 0, 1, 1 ).normalized() ),
	};
	EXPECT_FALSE( AxesIndependent( ToSpan( joints ) ) );
}

// ------------------------------------------------------------

}
