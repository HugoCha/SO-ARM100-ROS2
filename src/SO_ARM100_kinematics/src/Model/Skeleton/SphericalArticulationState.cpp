#include "Model/Skeleton/SphericalArticulationState.hpp"

#include "Global.hpp"
#include "Utils/MathUtils.hpp"

namespace SOArm100::Kinematics::Model
{

// ------------------------------------------------------------

SphericalArticulationState::SphericalArticulationState( ArticulationConstPtr articulation ) :
	ArticulationState( articulation )
{
	assert( articulation->GetType() == ArticulationType::Spherical );
}

// ------------------------------------------------------------

void SphericalArticulationState::ApplyConstraints( BoneState& bone_state ) const
{
}

// ------------------------------------------------------------

void SphericalArticulationState::UpdateValues( const BoneState& bone_state )
{
	auto joint_0 = articulation_->Joints()[0];
	auto joint_1 = articulation_->Joints()[1];
	auto joint_2 = articulation_->Joints()[2];

	Vec3d old_bone_direction = rotation_ * internal_rotation_ * bone_state.GetBone()->Direction();
	Vec3d new_bone_direction = bone_state.Direction();

	auto old_bone_to_new = 
		Quaternion::FromTwoVectors( old_bone_direction, new_bone_direction );

	Vec3d old_ref   = joint_states_[0]->Axis().cross( joint_states_[1]->Axis() );
	Vec3d intermediate_ref = old_ref;
	Vec3d final_ref = old_bone_to_new * old_ref;

	double old_value = joint_states_[0]->Value();
	double new_value = old_value + SignedAngle( intermediate_ref, final_ref, joint_states_[0]->Axis() );
	joint_states_[0]->Value() = joint_0->GetLimits().Clamp( new_value );
	
	auto joint_0_rotation = 
		AngleAxis( joint_states_[0]->Value() - old_value, joint_states_[0]->Axis() );
	intermediate_ref = joint_0_rotation * intermediate_ref;
	joint_states_[1]->Axis() = joint_0_rotation * joint_states_[1]->Axis();
	
	old_value = joint_states_[1]->Value();
	new_value = old_value + SignedAngle( intermediate_ref, final_ref, joint_states_[1]->Axis() );
	joint_states_[1]->Value() = joint_1->GetLimits().Clamp( new_value );
	
	auto joint_1_rotation = 
		AngleAxis( joint_states_[1]->Value() - old_value, joint_states_[1]->Axis() );
	intermediate_ref = joint_1_rotation * intermediate_ref;
	joint_states_[2]->Axis() = joint_1_rotation * joint_0_rotation * joint_states_[2]->Axis();

	old_value = joint_states_[2]->Value();
	new_value = old_value + SignedAngle( intermediate_ref, final_ref, joint_states_[2]->Axis() );
	joint_states_[2]->Value() = joint_2->GetLimits().Clamp( new_value );
}

// ------------------------------------------------------------

}