#include "Model/Space/ChainTotalLengthReachableSpace.hpp"

#include "Global.hpp"

#include "Model/Joint/JointChain.hpp"
#include "Utils/Distance.hpp"
#include "Utils/KinematicsUtils.hpp"

namespace SOArm100::Kinematics::Model
{

// ------------------------------------------------------------

ChainTotalLengthReachableSpace::ChainTotalLengthReachableSpace(
	const JointChain& chain,
	const Mat4d& home_configuration ) :
	SphericalReachableSpace( 
		ChainOrigin( chain ),
		ComputeTotalLength( chain, home_configuration ) )
{
}

// ------------------------------------------------------------

Vec3d ChainTotalLengthReachableSpace::ChainOrigin( const JointChain& chain )
{
	if ( chain.Empty() )
		return Vec3d::Zero();
	return chain.GetJoints()[0]->Origin();
}

// ------------------------------------------------------------

double ChainTotalLengthReachableSpace::ComputeTotalLength(
	const JointChain& chain,
	const Mat4d& home_configuration )
{
	if ( chain.Empty() )
		return 0.0;

	double total_length = 0.0;
	const auto& joints = chain.GetJoints();

	for ( const auto& joint : joints )
	{
		total_length += joint->GetLink().GetLength();
	}

	Vec3d p_last_joint = joints.back()->Origin();
	Vec3d p_tip = Translation( home_configuration );

	total_length += Utils::Distance( p_last_joint, p_tip );
	return total_length;
}

// ------------------------------------------------------------

}
