#include "Model/ReachableSpace/LineReachableSpace.hpp"

#include "Global.hpp"

#include "Utils/KinematicsUtils.hpp"
#include "Utils/MathUtils.hpp"

namespace SOArm100::Kinematics::Model
{

// ------------------------------------------------------------

bool LineReachableSpace::IsUnreachable( const Mat4d& target ) const
{
	Vec3d p_target = Translation( target );
	Vec3d diff = p_target - line_.point;
	return diff.cross( line_.axis ).norm() > epsilon;
}

// ------------------------------------------------------------

Mat4d LineReachableSpace::GetPossibleReachableTarget( const Mat4d& target ) const
{
	if ( !IsUnreachable( target ) )
		return target;

	Mat4d possible_target = target;

	Vec3d p_target = Translation( target );
	Vec3d p_projected = ProjectPointOnAxis( p_target, line_.point, line_.axis );
	possible_target.block< 3, 1 >( 0, 3 ) = p_projected;

	return possible_target;
}

// ------------------------------------------------------------

}