#include "Model/Skeleton/SkeletonState.hpp"

#include "Global.hpp"

#include "Model/Skeleton/ArticulationState.hpp"
#include "Model/Skeleton/ArticulationType.hpp"
#include "Model/Skeleton/BoneState.hpp"
#include "Model/Skeleton/PrismaticArticulationState.hpp"
#include "Model/Skeleton/RevoluteArticulationState.hpp"
#include "Model/Skeleton/Skeleton.hpp"
#include "Model/Skeleton/SphericalArticulationState.hpp"
#include "Model/Skeleton/UniversalArticulationState.hpp"
#include "Utils/Converter.hpp"

#include <memory>
#include <stdexcept>

namespace SOArm100::Kinematics::Model
{

// ------------------------------------------------------------

SkeletonState::SkeletonState( const Skeleton* skeleton ) :
	skeleton_( skeleton )
{
	for ( int i = 0; i < skeleton->ArticulationCount(); i++ )
	{
		articulation_states_.emplace_back( CreateArticulationState( skeleton->Articulation( i ).get() ) );
	}
}

// ------------------------------------------------------------

std::vector< BoneState > SkeletonState::GetBoneStates() const 
{
	std::vector< BoneState > bone_states;
	const auto& bones = skeleton_->Bones();

	for ( int i = 0; i < bones.size(); i++ )
	{
		const auto& bone = bones[i];
		const auto& articulation_state = articulation_states_[i];
		const auto& T_articulation = articulation_state->GlobalTransform();
		auto bone_state = BoneState( bones[i] );
		
		bone_state.Origin() = T_articulation.translation();
		bone_state.Direction() = T_articulation.rotation() * bone->Direction();

		bone_states.emplace_back( bone_state );
	}

	return bone_states;
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

	Iso3d world_transform = Iso3d::Identity();

	int sub_joint_idx = 0;
	for ( int i = 0; i < articulation_states_.size(); i++ )
	{
		int n_sub_joints = skeleton_->Articulation( i )->JointCount();
		VecXd sub_joints = joints.segment( sub_joint_idx, n_sub_joints );

		auto articulation_state = articulation_states_[i];

		articulation_state->SetState(
			world_transform,
			sub_joints );
		
		if ( i < articulation_states_.size() - 1 )
		{
			world_transform.translate( skeleton_->Bone( i )->Direction() );
			world_transform = articulation_state->LocalTransform() * world_transform;
			sub_joint_idx += n_sub_joints;
		}
	}
}

// ------------------------------------------------------------

void SkeletonState::UpdateValue( const BoneState& bone_state, int i )
{
	const int n_joints = skeleton_->ArticulationCount();
	if ( i < 0 || i >= n_joints ) throw std::out_of_range( "Index must match articulation index" );

	articulation_states_[i]->UpdateValues( bone_state );

	Iso3d transform; 

	for ( int j = i + 1; j < n_joints; j++ )
	{
		transform = articulation_states_[j-1]->GlobalTransform();
		transform.translate( skeleton_->Bone( j - 1 )->Direction() );
		articulation_states_[j]->SetCenterPose( transform );
	}
}

// ------------------------------------------------------------

void SkeletonState::ApplyConstraint( BoneState& bone_state, int i ) const
{
	const int n_joints = skeleton_->ArticulationCount();
	if ( i < 0 || i >= n_joints ) throw std::out_of_range( "Index must match articulation index" );

	articulation_states_[i]->ApplyConstraints( bone_state );
}

// ------------------------------------------------------------

ArticulationStatePtr SkeletonState::CreateArticulationState( const Articulation* articulation )
{
	switch ( articulation->GetType() )
	{
	case ArticulationType::Prismatic:
		return std::make_shared< PrismaticArticulationState >( articulation );
	case ArticulationType::Revolute:
		return std::make_shared< RevoluteArticulationState >( articulation );
	case ArticulationType::Universal:
		return std::make_shared< UniversalArticulationState >( articulation );
	case ArticulationType::Spherical:
		return std::make_shared< SphericalArticulationState >( articulation );
	default:
	{
		assert( "Unknown articulation type" );
		return nullptr;
	}
	}
}

// ------------------------------------------------------------

}