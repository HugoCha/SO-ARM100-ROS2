#pragma once

#include "Global.hpp"

#include "Model/ReachableSpace.hpp"
#include "Model/Skeleton.hpp"

namespace SOArm100::Kinematics::Model
{
class SkeletonTotalLengthReachableSpace : public ReachableSpace
{
public:
SkeletonTotalLengthReachableSpace( const Skeleton& skeleton ) :
    origin_( !skeleton.IsEmpty() ? 
        skeleton.Articulation( 0 )->Joints()[0]->Origin() : Vec3d::Zero() ),
    total_length_( skeleton.TotalLength() )
{}

inline virtual bool IsUnreachable( const Mat4d& target ) const override {
    return ( Translation( target ) - origin_ ).norm() > total_length_;
}

private:
Vec3d origin_;
double total_length_;
};
}