#include "Model/Skeleton/RevoluteArticulationBoneConstraint.hpp"

#include "Global.hpp"

#include "Model/Geometry/Base3d.hpp"
#include "Model/Skeleton/BoneState.hpp"

namespace SOArm100::Kinematics::Model
{

// ------------------------------------------------------------

RevoluteArticulationBoneConstraint::RevoluteArticulationBoneConstraint(
	ArticulationConstPtr articulation,
	BoneConstPtr bone ) :
	ArticulationBoneConstraint( articulation, bone )
{
	assert( articulation->GetType() == ArticulationType::Revolute );

	auto joint = articulation_->Joints()[0];

	Vec3d v_ref = bone->Direction();

	Vec3d z_ref = joint->Axis();
	z_ref.normalize();

	Vec3d x_ref = v_ref - v_ref.dot( joint->Axis() ) * joint->Axis();
	x_ref.normalize();

	Vec3d y_ref = z_ref.cross( x_ref );
	y_ref.normalize();

	base_ref_ = std::make_unique< const Base3d >( x_ref, y_ref, z_ref );
    bone_on_axis_ = x_ref.isZero();
}

// ------------------------------------------------------------

void RevoluteArticulationBoneConstraint::ApplyConstraint(
	const Quaternion& articulation_rotation,
	const Vec3d& articulation_center,
	BoneState& bone_state ) const
{
    if ( bone_on_axis_ )
    {
        bone_state.Direction() = Vec3d::Zero();
        return;
    }

	Vec3d v = bone_state.Direction();

	Base3d rotated_base = Rotate( *base_ref_, articulation_rotation );
	Vec3d v_plane = v - v.dot( rotated_base.z ) * rotated_base.z;

	if ( v_plane.norm() < epsilon )
	{
		bone_state.Direction() = bone_->Length() * rotated_base.z;
		return;
	}

	double angle = std::atan2( v_plane.dot( rotated_base.y ), v_plane.dot( rotated_base.x ) );

	auto joint = articulation_->Joints()[0];
	angle = joint->GetLimits().Clamp( angle );

	Vec3d bone_dir = ( rotated_base.x * cos( angle ) + rotated_base.y * sin( angle ) ).normalized();
	bone_state.Direction() = bone_->Length() * bone_dir;
}

// ------------------------------------------------------------

}