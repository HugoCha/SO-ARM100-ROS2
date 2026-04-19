#include "Model/Skeleton/RevoluteArticulationBoneConstraint.hpp"

#include "Global.hpp"

#include "Model/Geometry/Base3d.hpp"

namespace SOArm100::Kinematics::Model
{

// ------------------------------------------------------------

RevoluteArticulationBoneConstraint::RevoluteArticulationBoneConstraint(
	ArticulationConstPtr articulation,
	BoneConstPtr bone ) :
	ArticulationBoneConstraint( articulation, bone )
{
	assert( articulation->GetType() == ArticulationType::Revolute );
	assert( articulation->Axis() == bone->Direction() );

	auto joint = articulation_->Joints()[0];

	Vec3d v_ref = bone->Direction();

	Vec3d z_ref = joint->Axis();
	z_ref.normalize();

	Vec3d x_ref = v_ref - v_ref.dot( joint->Axis() ) * joint->Axis();
	x_ref.normalize();

	Vec3d y_ref = z_ref.cross( x_ref );
	y_ref.normalize();

	base_ref_ = std::make_unique< const Base3d >( x_ref, y_ref, z_ref );
}

// ------------------------------------------------------------

void RevoluteArticulationBoneConstraint::ApplyConstraint(
	const Quaternion& rotation,
	const Vec3d& center,
	Vec3d& direction ) const
{
	Vec3d v = direction;

	Base3d rotated_base = base_ref_->Rotate( rotation );
	Vec3d v_plane = v - v.dot( rotated_base.Z() ) * rotated_base.Z();

	if ( v_plane.norm() < epsilon )
	{
		direction = bone_->Length() * rotated_base.Z();
		return;
	}

	double angle = std::atan2( v_plane.dot( rotated_base.Y() ), v_plane.dot( rotated_base.X() ) );

	auto joint = articulation_->Joints()[0];
	angle = joint->GetLimits().Clamp( angle );

	Vec3d bone_dir = ( rotated_base.X() * cos( angle ) + rotated_base.Y() * sin( angle ) ).normalized();
	direction = bone_->Length() * bone_dir;
}

// ------------------------------------------------------------

}