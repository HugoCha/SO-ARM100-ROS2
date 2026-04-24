#include "Model/Skeleton/SkeletonState.hpp"

#include "Global.hpp"

#include "Model/Skeleton/ArticulationState.hpp"
#include "Model/Skeleton/ArticulationType.hpp"
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

SkeletonState::SkeletonState( SkeletonConstPtr skeleton ) :
	skeleton_( skeleton )
{
	for ( int i = 0; i < skeleton->ArticulationCount(); i++ )
	{
		articulation_states_.emplace_back( CreateArticulationState( skeleton->Articulation( i ) ) );
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

		world_transform.translate( skeleton_->Bone( i )->Direction() );
		world_transform = articulation_state->LocalTransform() * world_transform;

		sub_joint_idx += n_sub_joints;
	}
}

// ------------------------------------------------------------

ArticulationStatePtr SkeletonState::CreateArticulationState( const ArticulationConstPtr& articulation )
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