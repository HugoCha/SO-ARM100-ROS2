#include "Model/Space/SphericalReachableSpace.hpp"

#include "Global.hpp"

#include "Utils/Distance.hpp"
#include "Utils/KinematicsUtils.hpp"

namespace SOArm100::Kinematics::Model
{

// ------------------------------------------------------------

bool SphericalReachableSpace::IsUnreachable( const Mat4d& target ) const
{
    return Utils::Distance( Translation( target ), origin_ ) > radius_;
}

// ------------------------------------------------------------

Mat4d SphericalReachableSpace::GetPossibleReachableTarget( const Mat4d& target ) const
{
    if ( !IsUnreachable( target ) )
        return target;

    Mat4d possible_target = target;
    
    Vec3d p_target = Translation( target );
    Vec3d diff = p_target - origin_;
    Vec3d p_projected = origin_ + diff.normalized() * radius_;
    possible_target.block< 3, 1 >( 0, 3 ) = p_projected;

    return possible_target;
}

// ------------------------------------------------------------


}