#include "MatrixExponential.hpp"

#include "KinematicsUtils.hpp"
#include "Types.hpp"

namespace SOArm100::Kinematics
{
// ------------------------------------------------------------

MatrixExponential::MatrixExponential( const Twist& twist, double theta )
	: twist_( twist ), theta_( theta )
{
}

MatrixExponential::~MatrixExponential()
{
}

// ------------------------------------------------------------

Mat4d MatrixExponential::Compute() const
{
	Mat4d T = Mat4d::Identity();

	T.block< 3, 3 >( 0, 0 ) = ComputeRotation( theta_ );
	T.block< 3, 1 >( 0, 3 ) = ComputeTranslation( theta_ );
	// T.rotate( ComputeRotation( theta_ ) );
	// T.translate( ComputeTranslation( theta_ ) );

	return T;
}

// ------------------------------------------------------------

MatrixExponential::operator Mat4d () const
{
	return Compute();
}

// ------------------------------------------------------------

// MatrixExponential::operator Iso3d () const
// {
// 	return Compute().matrix();
// }

// ------------------------------------------------------------

Mat4d MatrixExponential::operator * ( const MatrixExponential& other ) const
{
	return Compute() * other.Compute();
}

// ------------------------------------------------------------

Mat4d MatrixExponential::operator * ( const Mat4d& matrix ) const
{
	return Compute() * matrix;
}

// ------------------------------------------------------------

Mat3d MatrixExponential::ComputeRotation( double theta ) const
{
	Vec3d w = twist_.GetAxis();
	Mat3d w_hat = SkewMatrix( w );

	Mat3d R = Mat3d::Identity() +
	          sin( theta ) * w_hat +
	          ( 1 - cos( theta ) ) * ( w_hat * w_hat );
	return R;
}

// ------------------------------------------------------------

Vec3d MatrixExponential::ComputeTranslation( double theta ) const
{
	Vec3d w = twist_.GetAxis();
	Vec3d v = twist_.GetLinear();
	Mat3d w_hat = SkewMatrix( w );

	Vec3d t = ( Mat3d::Identity() - ComputeRotation( theta ) ) *
	          ( w_hat * v ) +
	          w * w.transpose() * v * theta;
	return t;
}

// ------------------------------------------------------------

}
