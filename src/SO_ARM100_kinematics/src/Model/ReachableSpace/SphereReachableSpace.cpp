#include "Model/ReachableSpace/SphereReachableSpace.hpp"

#include "Global.hpp"

#include "Utils/Distance.hpp"
#include "Utils/KinematicsUtils.hpp"

namespace SOArm100::Kinematics::Model
{

// ------------------------------------------------------------

bool SphereReachableSpace::IsUnreachable( const Mat4d& target ) const
{
	return Utils::Distance( Translation( target ), sphere_.center ) > sphere_.radius;
}

// ------------------------------------------------------------

Mat4d SphereReachableSpace::GetPossibleReachableTarget( const Mat4d& target ) const
{
	if ( !IsUnreachable( target ) )
		return target;

	Mat4d possible_target = target;

	Vec3d p_target = Translation( target );
	Vec3d diff = p_target - sphere_.center;
	Vec3d p_projected = sphere_.center + diff.normalized() * sphere_.radius;
	possible_target.block< 3, 1 >( 0, 3 ) = p_projected;

	return possible_target;
}

// ------------------------------------------------------------

}