#pragma once

#include "Model/Skeleton/Skeleton.hpp"
#include "SphereReachableSpace.hpp"

namespace SOArm100::Kinematics::Model
{
class SkeletonTotalLengthReachableSpace : public SphereReachableSpace
{
public:
SkeletonTotalLengthReachableSpace( const Skeleton& skeleton ) :
	SphereReachableSpace(
		SkeletonOrigin( skeleton ),
		skeleton.TotalLength() )
{
}

private:
static Vec3d SkeletonOrigin( const Skeleton& skeleton )
{
	return !skeleton.IsEmpty() && !skeleton.Articulation( 0 )->IsEmpty() ?
	       skeleton.Articulation( 0 )->Joints()[0]->Origin() : Vec3d::Zero();
}
};
}