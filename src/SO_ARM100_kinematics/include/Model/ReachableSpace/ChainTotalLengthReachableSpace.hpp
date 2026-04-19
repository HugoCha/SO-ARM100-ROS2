#pragma once

#include "Global.hpp"

#include "SphereReachableSpace.hpp"

namespace SOArm100::Kinematics::Model
{
class JointChain;
}

namespace SOArm100::Kinematics::Model
{
class ChainTotalLengthReachableSpace : public SphereReachableSpace
{
public:
ChainTotalLengthReachableSpace( const JointChain& chain, const Mat4d& home_configuration );

private:
static Vec3d ChainOrigin( const JointChain& chain );

static double ComputeTotalLength(
	const JointChain& chain,
	const Mat4d& home_configuration );
};
}