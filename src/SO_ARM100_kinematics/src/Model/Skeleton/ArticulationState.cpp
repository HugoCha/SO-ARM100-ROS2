#include "Model/Skeleton/ArticulationState.hpp"

#include "Global.hpp"

#include "Model/Joint/JointState.hpp"
#include "Model/Skeleton/Articulation.hpp"
#include "Model/Skeleton/ArticulationType.hpp"

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
		joint_states_.push_back( std::make_shared< JointState >( *it ) );
	}
}

// ------------------------------------------------------------

std::vector< JointStateConstPtr > ArticulationState::GetJointStates() const
{
	std::vector< JointStateConstPtr > joint_states;
	joint_states.reserve( joint_states_.size() );

	for ( const auto& state : joint_states_ )
		joint_states.push_back( state );

	return joint_states;
}

// ------------------------------------------------------------

VecXd ArticulationState::GetJointValues() const
{
	VecXd joints( articulation_->JointCount() );

	for ( auto i = 0; i < joint_states_.size(); i++ )
		joints[i] = joint_states_[i]->Value();

	return joints;
}

// ------------------------------------------------------------

void ArticulationState::SetState(
	const Quaternion& rotation,
	const Vec3d& origin,
	const VecXd& values )
{
	center_ = origin;
	rotation_ = rotation;

	Quaternion internal_rotation = rotation;
	for ( auto i = 0; i < joint_states_.size(); i++ )
	{
		auto joint = articulation_->Joints()[i];
		auto joint_state = joint_states_[i];
		Vec3d translation = origin + internal_rotation * ( joint->Origin() - articulation_->Center() );
		Vec3d axis = internal_rotation * joint->Axis();
		joint_state->SetState( translation, axis, values[i] );
		internal_rotation = internal_rotation * AngleAxis( joint_state->Value(), joint->Axis() );
	}
}

// ------------------------------------------------------------

void ArticulationState::SetCenterPose( const Quaternion& rotation, const Vec3d& origin )
{
	center_ = origin;
	rotation_ = rotation;

	Quaternion internal_rotation = rotation;
	for ( auto i = 0; i < joint_states_.size(); i++ )
	{
		auto joint = articulation_->Joints()[i];
		auto joint_state = joint_states_[i];
		Vec3d translation = origin + internal_rotation * ( joint->Origin() - articulation_->Center() );
		Vec3d axis = internal_rotation * joint->Axis();
		joint_state->SetState( translation, axis, joint_state->Value() );
		internal_rotation = internal_rotation * AngleAxis( joint_state->Value(), joint->Axis() );
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
	Vec3d v = rotation_.inverse() * bone_state.Direction();

	auto joint = articulation_->Joints()[0];
	Vec3d v_plane = v - v.dot( joint->Axis() ) * joint->Axis();

	if ( v_plane.norm() < epsilon )
	{
		bone_state.Direction() = rotation_ * joint->Axis() * bone_state.GetBone()->Length();
		return;
	}

	Vec3d v_ref = bone_state.GetBone()->Direction();
	Vec3d x_ref = v_ref - v_ref.dot( joint->Axis() ) * joint->Axis();
	x_ref.normalize();
	Vec3d y_ref = joint->Axis().cross( x_ref );
	y_ref.normalize();

	double angle = std::atan2( v_plane.dot( y_ref ), v_plane.dot( x_ref ) );
	angle = joint->GetLimits().Clamp( angle );

	Vec3d bone_dir = ( x_ref * cos( angle ) + y_ref * sin( angle ) ).normalized();
	bone_state.Direction() = bone_state.GetBone()->Length() * bone_dir;
}

// ------------------------------------------------------------

void ArticulationState::ApplyUniversalConstraints( BoneState& bone_state ) const
{
	Vec3d v_proposed = bone_state.Direction();
	double L = bone_state.GetBone()->Length();

	Vec3d axis_pan = rotation_ * articulation_->Joints()[0]->Axis();
	Vec3d axis_tilt = rotation_ * articulation_->Joints()[1]->Axis();
	
	Vec3d z_ref = axis_pan.cross(axis_tilt).normalized(); 
	Vec3d x_ref = axis_pan.normalized();
	Vec3d y_ref = axis_tilt.normalized();

	double x = v_proposed.dot(x_ref);
	double y = v_proposed.dot(y_ref);
	double z = v_proposed.dot(z_ref);

	double alpha = std::atan2(x, z); 
	double beta  = std::atan2(y, z);

	Vec3d constrained_dir;
	if (std::abs(z) > epsilon)
		constrained_dir = (std::tan(alpha) * x_ref + std::tan(beta) * y_ref + z_ref).normalized();
	else
		constrained_dir = (std::sin(alpha) * x_ref + std::sin(beta) * y_ref).normalized();
	
	bone_state.Direction() = L * constrained_dir;
}

// ------------------------------------------------------------

void ArticulationState::ApplySphericalConstraints( BoneState& bone_state ) const
{
	Vec3d v = bone_state.Origin() + bone_state.Direction() - Origin();
	Vec3d z_ref = bone_state.GetBone()->Direction().normalized();

	auto it = joint_states_.begin();
	auto joint_1 = articulation_->Joints()[0];
	auto joint_state_1 = joint_states_[0];
	Vec3d x_ref = joint_1->Axis().cross( z_ref ).normalized();
	++it;

	auto joint_2 = articulation_->Joints()[1];
	auto joint_state_2 = joint_states_[1];
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
	auto joint = articulation_->Joints()[0];
	slide = joint->GetLimits().Clamp( slide );
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
		UpdatePrismaticArticulationValue( old_state, old_bone_state, new_bone_state );
		break;
	case ArticulationType::Revolute:
		UpdateRevoluteArticulationValue( old_state, old_bone_state, new_bone_state );
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
	const ArticulationState& old_state,
	const BoneState& old_bone_state,
	const BoneState& new_bone_state )
{
	assert( joint_states_.size() == 1 );

	auto joint_state = joint_states_[0];
	joint_state->UpdateValue(
		old_state.Origin(),
		old_bone_state.Direction(),
		new_bone_state.Direction() );
}

// ------------------------------------------------------------

void ArticulationState::UpdateRevoluteArticulationValue(
	const ArticulationState& old_state,
	const BoneState& old_bone_state,
	const BoneState& new_bone_state )
{
	assert( joint_states_.size() == 1 );

	auto joint_state = joint_states_[0];
	joint_state->UpdateValue(
		old_state.Origin(),
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

	Vec3d dir = rotation_.inverse() * new_bone_state.Direction();

	Vec3d axis_pan = articulation_->Joints()[0]->Axis();
	Vec3d axis_tilt = articulation_->Joints()[1]->Axis();

	Vec3d z_ref = axis_pan.cross( axis_tilt ).normalized();
	Vec3d x_ref = axis_pan.normalized();
	Vec3d y_ref = z_ref.cross( x_ref ).normalized();

	double x = dir.dot( x_ref );
	double y = dir.dot( y_ref );
	double z = dir.dot( z_ref );

	double alpha = std::atan2( x, z );
	double beta  = std::atan2( y, z );

	joint_states_[0]->Value() = articulation_->Joints()[0]->GetLimits().Clamp( alpha );
	joint_states_[1]->Value() = articulation_->Joints()[1]->GetLimits().Clamp( beta );
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