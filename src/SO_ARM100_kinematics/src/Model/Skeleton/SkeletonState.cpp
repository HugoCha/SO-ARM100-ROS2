#include "Model/Skeleton/SkeletonState.hpp"

#include "Global.hpp"

#include "Model/Skeleton/ArticulationState.hpp"
#include "Model/Skeleton/Skeleton.hpp"
#include "Utils/Converter.hpp"

#include <stdexcept>

namespace SOArm100::Kinematics::Model
{

// ------------------------------------------------------------

SkeletonState::SkeletonState( SkeletonConstPtr skeleton ) :
	skeleton_( skeleton )
{
	for ( int i = 0; i < skeleton->ArticulationCount(); i++ )
	{
		articulation_states_.emplace_back( std::make_shared< ArticulationState >( skeleton->Articulation( i ) ) );
	}
}

// ------------------------------------------------------------

VecXd SkeletonState::GetJointValues() const
{
	std::vector< double > joints( skeleton_->JointCount() );
	int joint_index = 0;
	for ( auto it = articulation_states_.begin(); it != articulation_states_.end(); ++it )
	{
		const auto& articulation_joints = ( *it )->GetJointValues();
		std::copy( articulation_joints.begin(), articulation_joints.end(), joints.begin() + joint_index );
		joint_index += articulation_joints.size();
	}
	return ToVecXd( joints );

}

// ------------------------------------------------------------

void SkeletonState::SetState( const VecXd& joints )
{
	const int n_joints = joints.size();
	if ( n_joints < skeleton_->JointCount() )
		throw std::invalid_argument( "Joints size mismatch" );

	Vec3d world_origin = Vec3d::Identity();
	Quaternion world_rotation = Quaternion::Identity();

	int sub_joint_idx = 0;
	for ( int i = 0; i < articulation_states_.size(); i++ )
	{
		int n_sub_joints = skeleton_->Articulation( i )->JointCount();
		VecXd sub_joints = joints.segment( sub_joint_idx, n_sub_joints );

		auto articulation_state = articulation_states_[i];

		articulation_state->SetState(
			world_rotation,
			world_origin,
			sub_joints );

		world_origin = articulation_state->Origin() + articulation_state->Axis() * skeleton_->Length( i );
		world_rotation = world_rotation * AngleAxis( articulation_state->Value(), articulation_state->Axis() );

		sub_joint_idx += n_sub_joints;
	}
}

// ------------------------------------------------------------

}