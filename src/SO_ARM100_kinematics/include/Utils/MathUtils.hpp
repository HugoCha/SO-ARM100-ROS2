#pragma once

#include <cmath>

namespace SOArm100::Kinematics
{

template< typename T >
T FindClosest( const T& target, const T& a, const T& b ){
	return ( std::abs( a - target ) < std::abs( b - target ) ) ? a : b;
}

}