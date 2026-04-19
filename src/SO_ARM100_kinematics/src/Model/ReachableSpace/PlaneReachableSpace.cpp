#include "Model/ReachableSpace/PlaneReachableSpace.hpp"

#include "Global.hpp"

#include "Utils/KinematicsUtils.hpp"
#include "Utils/MathUtils.hpp"

namespace SOArm100::Kinematics::Model
{

// ------------------------------------------------------------

bool PlaneReachableSpace::IsUnreachable( const Mat4d& target ) const
{
	Vec3d p_target = Translation( target );
	Vec3d diff = p_target - plane_.point;
	return diff.dot( plane_.normal ) < 0;
}

// ------------------------------------------------------------

Mat4d PlaneReachableSpace::GetPossibleReachableTarget( const Mat4d& target ) const
{
	if ( !IsUnreachable( target ) )
		return target;

	Mat4d possible_target = target;

	Vec3d p_target = Translation( target );
	Vec3d p_projected = ProjectPointOnPlane( p_target, plane_.point, plane_.normal );
	possible_target.block< 3, 1 >( 0, 3 ) = p_projected;

	return possible_target;
}

// ------------------------------------------------------------

}