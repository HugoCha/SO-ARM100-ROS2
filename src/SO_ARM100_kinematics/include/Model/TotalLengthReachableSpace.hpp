#pragma once

#include "Global.hpp"

#include "Model/ReachableSpace.hpp"

namespace SOArm100::Kinematics::Model
{
class JointChain;
}

namespace SOArm100::Kinematics::Model
{
class TotalLengthReachableSpace : public ReachableSpace
{
public:
TotalLengthReachableSpace( const JointChain& chain, const Mat4d& home_configuration );

virtual bool IsUnreachable( const Mat4d& target ) const override;

private:
Vec3d origin_;
double total_length_;

static Vec3d ChainOrigin( const JointChain& chain );

static double ComputeTotalLength(
	const JointChain& chain,
	const Mat4d& home_configuration );
};
}