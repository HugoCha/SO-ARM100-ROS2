#include "Utils/Distance.hpp"

#include <stdexcept>

namespace SOArm100::Kinematics::Utils
{

double Distance(
	const VecXd& p1,
	const VecXd& p2,
	DistanceType type )
{
	if ( p1.size() != p2.size() )
		throw std::invalid_argument( "Vector size mismatch" );

	switch ( type )
	{
		case DistanceType::Chebysev:
			return ( p1 - p2 ).maxCoeff();
		case DistanceType::Manhattan:
			return ( p1 - p2 ).cwiseAbs().sum();
		default:
		case DistanceType::Euclidean:
			return ( p1 - p2 ).norm();
	}
}

}