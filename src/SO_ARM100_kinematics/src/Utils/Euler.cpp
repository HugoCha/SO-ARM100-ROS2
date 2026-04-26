#include "Utils/Euler.hpp"

#include "Global.hpp"

#include <map>
#include <optional>

namespace SOArm100::Kinematics
{

// ------------------------------------------------------------

std::optional< Euler::Configuration > Euler::ComputeConfiguration(
	const Vec3d& a1,
	const Vec3d& a2,
	const Vec3d& a3 )
{
	Vec3d a1_normalized = a1.normalized();
	Vec3d a2_normalized = a2.normalized();
	Vec3d a3_normalized = a3.normalized();

	if ( a1_normalized.dot( a2_normalized ) > epsilon ||
	     a2_normalized.dot( a3_normalized ) > epsilon )
	{
		return std::nullopt;
	}

	auto dominant = []( const Vec3d& v ) -> int {
						int i = 0;
						v.cwiseAbs().maxCoeff( & i );
						return i; };

	int i1 = dominant( a1 );
	int i2 = dominant( a2 );
	int i3 = dominant( a3 );

	static const std::map< std::tuple< int, int, int >, Configuration > table {
		{{ 0, 1, 2 }, Configuration::XYZ }, {{ 0, 2, 1 }, Configuration::XZY },
		{{ 1, 0, 2 }, Configuration::YXZ }, {{ 1, 2, 0 }, Configuration::YZX },
		{{ 2, 0, 1 }, Configuration::ZXY }, {{ 2, 1, 0 }, Configuration::ZYX },
	};

	auto key = std::make_tuple( i1, i2, i3 );
	auto it  = table.find( key );
	if ( it == table.end() )
    {
		return std::nullopt;
    }

	return it->second;
}

// ------------------------------------------------------------

Vec3d Euler::Solve(
	Configuration configuration,
	const Vec3d& a1,
	const Vec3d& a2,
	const Vec3d& a3,
	const Vec3d& old_dir,
	const Vec3d& new_dir )
{
	Vec3d a1_normalized = a1.normalized();
	Vec3d a2_normalized = a2.normalized();
	Vec3d a3_normalized = a3.normalized();

	assert( ( a1_normalized.dot( a2_normalized ) ) < epsilon );
	assert( ( a2_normalized.dot( a3_normalized ) ) < epsilon );

	Mat3d P;
	P << a1_normalized, a2_normalized, a3_normalized;
	Mat3d R_total = Quaternion::FromTwoVectors( old_dir, new_dir ).toRotationMatrix();
	Mat3d R_local = P.transpose() * R_total * P;

	return SolveFromRotation( configuration, R_local );
}

// ------------------------------------------------------------

Vec3d Euler::SolveFromRotation( Configuration configuration, Mat3d rotation )
{
	double a, b, c;

	switch ( configuration )
	{
	case Configuration::ZYX:
	{
		b = std::atan2( -rotation( 2, 0 ), std::sqrt( rotation( 0, 0 ) * rotation( 0, 0 ) + rotation( 1, 0 ) * rotation( 1, 0 ) ) );
		if ( std::abs( std::cos( b ) ) < 1e-6 )     // gimbal lock
		{
			a = 0;
			c = std::atan2( rotation( 0, 1 ), rotation( 1, 1 ) );
		}
		else
		{
			a = std::atan2( rotation( 2, 1 ), rotation( 2, 2 ) );
			c = std::atan2( rotation( 1, 0 ), rotation( 0, 0 ) );
		}
		break;
	}
	case Configuration::XYZ:
	{
		b = std::atan2( rotation( 0, 2 ), std::sqrt( rotation( 1, 2 ) * rotation( 1, 2 ) + rotation( 2, 2 ) * rotation( 2, 2 ) ) );
		if ( std::abs( std::cos( b ) ) < 1e-6 )
		{
			a = std::atan2( rotation( 1, 0 ), rotation( 1, 1 ) ); c = 0;
		}
		else
		{
			a = std::atan2( -rotation( 1, 2 ), rotation( 2, 2 ) );
			c = std::atan2( -rotation( 0, 1 ), rotation( 0, 0 ) );
		}
		break;
	}
	case Configuration::YZX:
	{
		b = std::atan2( rotation( 1, 0 ), std::sqrt( rotation( 0, 0 ) * rotation( 0, 0 ) + rotation( 2, 0 ) * rotation( 2, 0 ) ) );
		if ( std::abs( std::cos( b ) ) < 1e-6 )
		{
			a = std::atan2( rotation( 0, 2 ), rotation( 0, 1 ) ); c = 0;
		}
		else
		{
			a = std::atan2( -rotation( 2, 0 ), rotation( 0, 0 ) );
			c = std::atan2( -rotation( 1, 2 ), rotation( 1, 1 ) );
		}
		break;
	}
	case Configuration::ZXY:
	{
		b = std::atan2( rotation( 2, 1 ), std::sqrt( rotation( 0, 1 ) * rotation( 0, 1 ) + rotation( 1, 1 ) * rotation( 1, 1 ) ) );
		if ( std::abs( std::cos( b ) ) < 1e-6 )
		{
			a = std::atan2( rotation( 0, 2 ), rotation( 0, 0 ) ); c = 0;
		}
		else
		{
			a = std::atan2( -rotation( 0, 1 ), rotation( 1, 1 ) );
			c = std::atan2( -rotation( 2, 0 ), rotation( 2, 2 ) );
		}
		break;
	}
	case Configuration::XZY:
	{
		b = std::atan2( -rotation( 0, 1 ), std::sqrt( rotation( 0, 0 ) * rotation( 0, 0 ) + rotation( 0, 2 ) * rotation( 0, 2 ) ) );
		if ( std::abs( std::cos( b ) ) < 1e-6 )
		{
			a = std::atan2( -rotation( 2, 0 ), rotation( 2, 2 ) ); c = 0;
		}
		else
		{
			a = std::atan2( rotation( 2, 1 ), rotation( 1, 1 ) );
			c = std::atan2( rotation( 0, 2 ), rotation( 0, 0 ) );
		}
		break;
	}
	case Configuration::YXZ:
	default:
	{
		b = std::atan2( -rotation( 1, 2 ), std::sqrt( rotation( 0, 2 ) * rotation( 0, 2 ) + rotation( 2, 2 ) * rotation( 2, 2 ) ) );
		if ( std::abs( std::cos( b ) ) < 1e-6 )
		{
			a = std::atan2( rotation( 0, 1 ), rotation( 0, 0 ) ); c = 0;
		}
		else
		{
			a = std::atan2( rotation( 0, 2 ), rotation( 2, 2 ) );
			c = std::atan2( rotation( 1, 0 ), rotation( 1, 1 ) );
		}
		break;
	}
	}
	return { a, b, c };
}

// ------------------------------------------------------------

}