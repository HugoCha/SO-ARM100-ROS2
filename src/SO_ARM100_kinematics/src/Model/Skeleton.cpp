#include "Model/Skeleton.hpp"

#include "Global.hpp"

#include "Model/Articulation.hpp"
#include "Model/KinematicModel.hpp"

#include <memory>
#include <stdexcept>
#include <vector>

namespace SOArm100::Kinematics::Model
{

// ------------------------------------------------------------

Skeleton Skeleton::Create(
	const std::span< const JointConstPtr >& joints,
	const Mat4d& tip )
{
	auto articulations = Articulation::ExtractFromJoints( joints, tip );
	auto bones = ComputeBones( articulations, tip );
	auto total_length = ComputeTotalLength( articulations, bones, tip );

	return Skeleton{ articulations, bones, total_length, ( int )joints.size() };
}

// ------------------------------------------------------------

Skeleton Skeleton::CreateFromKinematicModel( KinematicModelConstPtr model )
{
	return Create( model->GetChain()->GetActiveJoints(), model->GetHomeConfiguration() );
}

// ------------------------------------------------------------

Skeleton Skeleton::CreateFromKinematicModel( KinematicModelConstPtr model, const Model::JointGroup& group )
{
	auto group_joints = ExtractGroupJoints( *model->GetChain(), group );
	return Create( group_joints, group.tip_home );
}

// ------------------------------------------------------------

std::vector< BoneConstPtr > Skeleton::ComputeBones(
	const std::vector< ArticulationConstPtr >& articulations,
	const Mat4d& tip )
{
	const int n_articulations = articulations.size();

	std::vector< BoneConstPtr > bones;
	for ( int i = 0; i < n_articulations - 1; i++ )
	{
		auto bone = articulations[i + 1]->Center() - articulations[i]->Center();
		bones.emplace_back( std::make_shared< Model::Bone >( articulations[i]->Center(), bone ) );
	}

	auto p_tip = Translation( tip );
	Vec3d last_bone = p_tip - articulations[n_articulations - 1]->Center();
	if ( last_bone.norm() > epsilon )
	{
		bones.emplace_back( std::make_shared< Model::Bone >( articulations[n_articulations - 1]->Center(), last_bone ) );
	}

	return bones;
}

// ------------------------------------------------------------

double Skeleton::ComputeTotalLength(
	const std::span< const ArticulationConstPtr >& articulations,
	const std::span< const BoneConstPtr >& bones,
	const Mat4d& tip )
{
	double total_length = 0.0;

	if ( !articulations.empty() )
	{
		const auto& first_articulation = articulations.front();
		const auto& first_joint = first_articulation->Joints().front();

		double initial_offset =
			( first_articulation->Center() - first_joint->Origin() ).norm();

		total_length += initial_offset;

		const auto& final_articulation = articulations.back();
		const auto& p_tip = Translation( tip );
		double final_offset =
			( p_tip - final_articulation->Center() ).norm();

		total_length += final_offset;
	}

	for ( int i = 0; i < bones.size(); i++ )
		total_length += bones[i]->Length();

	return total_length;
}

// ------------------------------------------------------------

std::vector< JointConstPtr > Skeleton::ExtractGroupJoints(
	const Model::JointChain& chain,
	const Model::JointGroup& group )
{
	if ( group.FirstIndex() < 0 || group.LastIndex() >= chain.GetActiveJointCount() )
	{
		throw std::invalid_argument( "Invalid group joint size" );
	}

	std::vector< JointConstPtr > group_joints( group.Size() );
	for ( auto it = group.indices.begin(); it != group.indices.end(); ++it )
	{
		group_joints.emplace_back( chain.GetActiveJoint( *it ) );
	}
	return group_joints;
}

// ------------------------------------------------------------



// ------------------------------------------------------------

}