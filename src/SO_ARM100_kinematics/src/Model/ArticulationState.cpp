#include "Model/ArticulationState.hpp"

#include "Global.hpp"

#include "Model/Articulation.hpp"
#include "Model/ArticulationType.hpp"
#include "Model/Joint.hpp"
#include "Model/JointState.hpp"
#include "Utils/Converter.hpp"
#include "Utils/MathUtils.hpp"

#include <cmath>
#include <memory>

namespace SOArm100::Kinematics::Model
{

// ------------------------------------------------------------

ArticulationState::ArticulationState( ArticulationConstPtr articulation ) :
	articulation_( articulation ),
	center_( articulation->Center() ),
	rotation_( Quaternion::Identity() )
{
	for ( auto it = articulation_->Joints().begin(); it != articulation->Joints().end(); ++it )
	{
		joint_states_.insert( { *it, std::make_shared< JointState >( *it ) } );
	}
}

// ------------------------------------------------------------

VecXd ArticulationState::GetJointValues() const
{
	std::vector< double > joints( articulation_->JointCount() );
	for ( auto it = joint_states_.begin(); it != joint_states_.end(); ++it )
	{
		auto joint_state = it->second;
		joints.emplace_back( joint_state->Value() );
	}
	return ToVecXd( joints );
}

// ------------------------------------------------------------

void ArticulationState::SetState(
	const Quaternion rotation,
	const Vec3d& origin,
	const VecXd& values )
{
	center_.origin = origin;
	center_.axis = rotation * articulation_->Center();
	rotation_ = rotation;

	int value_idx = 0;

	for ( auto it = joint_states_.begin(); it != joint_states_.end(); it++ )
	{
		auto joint = it->first;
		auto joint_state = it->second;
		Vec3d translation = origin + rotation * ( joint->Origin() - articulation_->Center() );
		Vec3d axis = rotation * joint->Axis();
		joint_state->SetState( axis, translation, values[value_idx++] );
	}
}

// ------------------------------------------------------------

void ArticulationState::SetPose( const Quaternion& rotation, const Vec3d& origin )
{
	center_.origin = origin;
	center_.axis = rotation * articulation_->Center();
	rotation_ = rotation;

	for ( auto it = joint_states_.begin(); it != joint_states_.end(); it++ )
	{
		auto joint = it->first;
		auto joint_state = it->second;
		Vec3d translation = origin + rotation * ( joint->Origin() - articulation_->Center() );
		Vec3d axis = rotation * joint->Axis();
		joint_state->SetPose( axis, translation );
	}
}

// ------------------------------------------------------------

void ArticulationState::ApplyConstraints( BoneState& bone_state ) const
{
	switch ( articulation_->GetType() )
	{
	case ArticulationType::Prismatic:
	{
		ApplyPrismaticConstraints( bone_state );
		return;
	}
	case ArticulationType::Revolute:
	{
		ApplyRevoluteConstraints( bone_state );
		return;
	}
	case ArticulationType::Universal:
	{
		ApplyUniversalConstraints( bone_state );
		return;
	}
	case ArticulationType::Spherical:
	{
		ApplySphericalConstraints( bone_state );
		return;
	}
	default:
	{
		bone_state.Direction().setZero();
		return;
	}
	}
}

// ------------------------------------------------------------

void ArticulationState::ApplyRevoluteConstraints( BoneState& bone_state ) const
{
	Vec3d v = bone_state.Origin() + bone_state.Direction() - Origin();

	Vec3d v_plane = v - v.dot( Axis() ) * Axis();

	if ( v_plane.norm() < epsilon )
	{
		bone_state.Direction() = bone_state.GetBone()->Length() * Axis();
		return;
	}

	Vec3d v_ref = bone_state.GetBone()->Origin() + bone_state.GetBone()->Direction() - articulation_->Center();
	Vec3d x_ref = v_ref - v_ref.dot( articulation_->Axis() ) * articulation_->Axis();
	x_ref.normalize();
	Vec3d y_ref = articulation_->Axis().cross( x_ref );
	y_ref.normalize();

	double angle = std::atan2( v_plane.dot( y_ref ), v_plane.dot( x_ref ) );
	angle = joint_states_.begin()->first->GetLimits().Clamp( angle );

	Vec3d bone_dir = ( x_ref * cos( angle ) + y_ref * sin( angle ) ).normalized();
	bone_state.Direction() = bone_state.GetBone()->Length() * bone_dir;
}

// ------------------------------------------------------------

void ArticulationState::ApplyUniversalConstraints( BoneState& bone_state ) const
{
	Vec3d v = bone_state.Origin() + bone_state.Direction() - Origin();
	Vec3d v_ref = bone_state.GetBone()->Origin() + bone_state.GetBone()->Direction() - articulation_->Center();

	auto it = joint_states_.begin();
	auto joint_1 = it->first;
	auto joint_state_1 = it->second;
	Vec3d x_ref = v_ref - v_ref.dot( joint_1->Axis() ) * joint_1->Axis();
	x_ref.normalize();
	++it;

	auto joint_2 = it->first;
	auto joint_state_2 = it->second;
	Vec3d y_ref = v_ref - v_ref.dot( joint_2->Axis() ) * joint_2->Axis();
	y_ref.normalize();

	Vec3d z_ref = x_ref.cross( y_ref );

	double x = v.dot( x_ref );
	double y = v.dot( y_ref );
	double z = v.dot( z_ref );

	double alpha = atan2( x, z );
	double beta  = atan2( y, z );

	alpha = joint_1->GetLimits().Clamp( alpha );
	beta  = joint_2->GetLimits().Clamp( beta );

	Vec3d bone_dir = ( tan( alpha ) * x_ref + tan( beta ) * y_ref + z_ref ).normalized();
	bone_state.Direction() = bone_state.GetBone()->Length() * bone_dir;
}

// ------------------------------------------------------------

void ArticulationState::ApplySphericalConstraints( BoneState& bone_state ) const
{
	Vec3d v = bone_state.Origin() + bone_state.Direction() - Origin();
	Vec3d z_ref = bone_state.GetBone()->Direction().normalized();

	auto it = joint_states_.begin();
	auto joint_1 = it->first;
	auto joint_state_1 = it->second;
	Vec3d x_ref = joint_1->Axis().cross( z_ref ).normalized();
	++it;

	auto joint_2 = it->first;
	auto joint_state_2 = it->second;
	Vec3d y_ref = z_ref.cross( x_ref ).normalized();

	double x = v.dot( x_ref );
	double y = v.dot( y_ref );
	double z = v.dot( z_ref );

	double alpha = atan2( x, z );
	double beta  = atan2( y, z );

	alpha = joint_1->GetLimits().Clamp( alpha );
	beta  = joint_2->GetLimits().Clamp( beta );

	Vec3d bone_dir = ( tan( alpha ) * x_ref + tan( beta ) * y_ref + z_ref ).normalized();
	bone_state.Direction() = bone_state.GetBone()->Length() * bone_dir;
}

// ------------------------------------------------------------

void ArticulationState::ApplyPrismaticConstraints( BoneState& bone_state ) const
{
	double slide = Axis().dot( bone_state.Direction() );
	slide = joint_states_.rbegin()->first->GetLimits().Clamp( slide );
	bone_state.Direction() = slide * Axis();
}

// ------------------------------------------------------------

void ArticulationState::UpdateValue(
	const ArticulationState& old_state,
	const BoneState& old_bone_state,
	const BoneState& new_bone_state )
{
	switch ( articulation_->GetType() )
	{
	case ArticulationType::Prismatic:
		UpdatePrismaticArticulationValue( old_bone_state, new_bone_state );
		break;
	case ArticulationType::Revolute:
		UpdateRevoluteArticulationValue( old_bone_state, new_bone_state );
		break;
	case ArticulationType::Universal:
		UpdateUniversalArticulationValue( old_state, old_bone_state, new_bone_state );
		break;
	case ArticulationType::Spherical:
		UpdateSphericalArticulationValue( old_state, old_bone_state, new_bone_state );
		break;
	default:
		break;
	}
}

// ------------------------------------------------------------

void ArticulationState::UpdatePrismaticArticulationValue(
	const BoneState& old_bone_state,
	const BoneState& new_bone_state )
{
	assert( joint_states_.size() == 1 );

	auto joint_state = joint_states_.begin()->second;
	joint_state->UpdateValue(
		old_bone_state.Origin(),
		old_bone_state.Direction(),
		new_bone_state.Direction() );
}

// ------------------------------------------------------------

void ArticulationState::UpdateRevoluteArticulationValue(
	const BoneState& old_bone_state,
	const BoneState& new_bone_state )
{
	assert( joint_states_.size() == 1 );

	auto joint_state = joint_states_.begin()->second;
	joint_state->UpdateValue(
		old_bone_state.Origin(),
		old_bone_state.Direction(),
		new_bone_state.Direction() );
}

// ------------------------------------------------------------

void ArticulationState::UpdateUniversalArticulationValue(
	const ArticulationState& old_state,
	const BoneState& old_bone_state,
	const BoneState& new_bone_state )
{
	assert( joint_states_.size() == 2 );

	Vec3d v_old = old_bone_state.Direction();
	Vec3d v_new = new_bone_state.Direction();

	auto it = joint_states_.begin();
	auto joint_1 = it->first;
	auto joint_1_state = it->second;
	++it;
	auto joint_2 = it->first;
	auto joint_2_state = it->second;

	Vec3d axis_1_old = old_state.joint_states_.at( joint_1 )->Axis();
	Vec3d axis_2_old = old_state.joint_states_.at( joint_2 )->Axis();
	Vec3d axis_1_new = joint_1_state->Axis();
	Vec3d axis_2_new = joint_2_state->Axis();

	Vec3d dir_1_old = v_old - v_old.dot( axis_2_old ) * axis_2_old;
	Vec3d dir_1_new = v_new - v_new.dot( axis_2_new ) * axis_2_new;

	double old_value = joint_1_state->Value();

	joint_1_state->UpdateValue(
		old_bone_state.Origin(),
		dir_1_old,
		dir_1_new );

	auto intermediate_aa = Quaternion::FromTwoVectors( axis_1_old, axis_1_new );
	auto joint_1_aa = AngleAxis( joint_1_state->Value() - old_value, joint_1_state->Axis() );
	Vec3d v_intermediate = intermediate_aa * joint_1_aa * v_old;

	joint_2_state->UpdateValue(
		old_state.Origin(),
		v_intermediate,
		v_new );
}

// ------------------------------------------------------------

void ArticulationState::UpdateSphericalArticulationValue(
	const ArticulationState& old_state,
	const BoneState& old_bone_state,
	const BoneState& new_bone_state )
{
	assert( joint_states_.size() == 3 );

	/*
	   auto articulation_aa = ComputeArticulationAngleAxis( old_state );
	   auto articulation_translation = ComputeArticulationTranslation( old_state );
	   auto articulation_center = old_state.Origin();

	   auto it = joint_states_.begin();
	   const JointConstPtr& joint_1 = it->first;
	   JointStatePtr joint_1_state = joint_states_.at( joint_1 );
	   const JointStatePtr old_joint_1_state = old_state.joint_states_.at( joint_1 );
	 ++it;

	   const JointConstPtr& joint_2 = it->first;
	   JointStatePtr joint_2_state = joint_states_.at( joint_2 );
	   const JointStatePtr old_joint_2_state = old_state.joint_states_.at( joint_2 );
	 ++it;

	   const JointConstPtr& joint_3 = it->first;
	   JointStatePtr joint_3_state = joint_states_.at( joint_3 );
	   const JointStatePtr old_joint_3_state = old_state.joint_states_.at( joint_3 );

	   Vec3d old_ref = old_joint_1_state->Axis().cross( old_joint_2_state->Axis() );
	   Vec3d final_ref = articulation_aa * old_ref;
	   Vec3d intermediate_axis = old_joint_1_state->Axis();

	   joint_1_state->RotateAround( articulation_aa, articulation_center );
	   joint_1_state->Translate( articulation_translation );

	   UpdateRevoluteJointValue(
	    old_ref,
	    final_ref,
	    intermediate_axis,
	    joint_1->GetLimits(),
	    joint_1_state );

	   auto joint_1_aa = AngleAxis( joint_1_state->Value(), joint_1_state->Axis() );
	   old_ref = joint_1_aa * old_ref;
	   intermediate_axis = joint_1_aa * old_joint_2_state->Axis();
	   articulation_aa = articulation_aa * joint_1_aa;

	   joint_2_state->RotateAround( articulation_aa, articulation_center );
	   joint_2_state->Translate( articulation_translation );

	   UpdateRevoluteJointValue(
	    old_ref,
	    final_ref,
	    intermediate_axis,
	    joint_2->GetLimits(),
	    joint_2_state );

	   auto joint_2_aa = AngleAxis( joint_2_state->Value(), joint_2_state->Axis() );
	   old_ref = joint_2_aa * old_ref;
	   intermediate_axis = joint_1_aa * joint_2_aa * old_joint_3_state->Axis();
	   articulation_aa = articulation_aa * joint_2_aa;

	   joint_3_state->RotateAround( articulation_aa, articulation_center );
	   joint_3_state->Translate( articulation_translation );

	   UpdateRevoluteJointValue(
	    old_ref,
	    final_ref,
	    intermediate_axis,
	    joint_3->GetLimits(),
	    joint_3_state );
	 */
}

// ------------------------------------------------------------

}