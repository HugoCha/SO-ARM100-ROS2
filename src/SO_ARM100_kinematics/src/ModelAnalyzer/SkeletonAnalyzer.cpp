#include "ModelAnalyzer/SkeletonAnalyzer.hpp"

#include "Global.hpp"

#include "Model/Joint/Joint.hpp"
#include "Model/Joint/JointGroup.hpp"
#include "Model/Skeleton/Articulation.hpp"
#include "Model/Skeleton/ArticulationType.hpp"
#include "Model/Skeleton/Skeleton.hpp"
#include "Utils/KinematicsUtils.hpp"

#include <memory>

namespace SOArm100::Kinematics::Model
{

// ------------------------------------------------------------

SkeletonConstPtr SkeletonAnalyzer::Analyze(
	const std::span< const JointConstPtr >& joints,
	const Mat4d& tip )
{
	auto articulations = AnalyzeArticulations( joints, tip );
	auto bones = ComputeBones( articulations, tip );
	auto total_length = ComputeTotalLength( articulations, bones, tip );

	return std::make_shared< const Skeleton >(
		articulations,
		bones,
		total_length,
		( int )joints.size() );
}

// ------------------------------------------------------------

SkeletonConstPtr SkeletonAnalyzer::Analyze(
	const std::span< const JointConstPtr >& joints,
	const Model::JointGroup& group )
{
	auto group_joints = ExtractGroupJoints( joints, group );
	return Analyze( group_joints, group.tip_home );
}

// ------------------------------------------------------------

std::vector< BoneConstPtr > SkeletonAnalyzer::ComputeBones(
	const std::span< const ArticulationConstPtr >& articulations,
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

double SkeletonAnalyzer::ComputeTotalLength(
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

		if ( bones.size() < articulations.size() )
		{
			const auto& final_articulation = articulations.back();
			const auto& p_tip = Translation( tip );
			double final_offset =
				( p_tip - final_articulation->Center() ).norm();

			total_length += final_offset;
		}
	}

	for ( int i = 0; i < bones.size(); i++ )
		total_length += bones[i]->Length();

	return total_length;
}

// ------------------------------------------------------------

std::vector< JointConstPtr > SkeletonAnalyzer::ExtractGroupJoints(
	const std::span< const JointConstPtr >& joints,
	const Model::JointGroup& group )
{
	if ( group.FirstIndex() < 0 || group.LastIndex() >= joints.size() )
	{
		throw std::invalid_argument( "Invalid group joint size" );
	}

	std::vector< JointConstPtr > group_joints( group.Size() );
	int i = 0;
	for ( auto it = group.indices.begin(); it != group.indices.end(); ++it )
	{
		group_joints[i++] = joints[*it];
	}
	return group_joints;
}

// ------------------------------------------------------------

std::vector< std::shared_ptr< const Articulation >> SkeletonAnalyzer::AnalyzeArticulations(
	const std::span< const JointConstPtr >& raw_joints,
	const Mat4d& tip )
{
	auto joints = FilterJoints( raw_joints );
	const int n_joints = joints.size();

	std::vector< std::shared_ptr< const Articulation >> articulations;

	auto last_articulation = ExtractArticulationFromLastJoints( joints, tip );
	const int n_joints_without_last = n_joints - last_articulation->JointCount();

	int index = n_joints_without_last - 1;
	while ( index >= 0 )
	{
		if ( index >= 2 && joints[index - 2]->GetLink().GetLength() > 0 )
		{
			if ( auto articulation = ExtractArticulationFromJoints(
					 joints[index],
					 joints[index - 1],
					 joints[index - 2] ) )
			{
				index -= articulation->JointCount();
				articulations.emplace( articulations.begin(), std::move( articulation ) );
				continue;
			}
		}
		if ( index >= 1 && joints[index - 1]->GetLink().GetLength() > 0  )
		{
			if ( auto articulation = ExtractArticulationFromJoints(
					 joints[index],
					 joints[index - 1] ) )
			{
				index -= articulation->JointCount();
				articulations.emplace( articulations.begin(), std::move( articulation ) );
				continue;
			}
		}
		if ( index >= 0 )
		{
			if ( auto articulation = ExtractArticulationFromJoints(
					 joints[index] ) )
			{
				articulations.emplace( articulations.begin(), std::move( articulation ) );
			}
			index -= 1;
		}
	}

	articulations.push_back( std::move( last_articulation ) );
	return articulations;
}

// ------------------------------------------------------------

std::vector< JointConstPtr > SkeletonAnalyzer::FilterJoints( const std::span< const JointConstPtr >& joints )
{
	const int n_joints = joints.size();
	if ( n_joints == 1 )
		return { joints[0] }
	;

	std::vector< JointConstPtr > filtered_joints;
	int i = 0;

	auto can_filter_joint = []( JointConstPtr joint1, JointConstPtr joint2 ) -> bool
							{
								if ( !joint1->Axis().isApprox( joint2->Axis() ) )
									return false;

								Vec3d direction = joint2->Origin() - joint1->Origin();
								bool is_direction_colinear = joint1->Axis().cross( direction ).norm() < epsilon;
								return is_direction_colinear;
							};

	while ( i + 1 < n_joints )
	{
		int j = 1;
		while ( i + j < n_joints && can_filter_joint( joints[i], joints[i + j] ) )
			j++;

		filtered_joints.emplace_back( joints[i] );
		i += j;
	}

	return filtered_joints;
}

// ------------------------------------------------------------

std::shared_ptr< Articulation > SkeletonAnalyzer::ExtractArticulationFromJoints(
	JointConstPtr joint1 )
{
	if ( !joint1 || joint1->IsFixed() )
		return nullptr;

	if ( joint1->IsPrismatic() )
	{
		return std::shared_ptr< Articulation >( new Articulation(
													ArticulationType::Prismatic,
													std::vector< JointConstPtr >{ joint1 },
													joint1->Origin() ) );
	}

	return std::shared_ptr< Articulation >( new Articulation(
												ArticulationType::Revolute,
												std::vector< JointConstPtr >{ joint1 },
												joint1->Origin() ) );
}

// ------------------------------------------------------------

std::shared_ptr< Articulation > SkeletonAnalyzer::ExtractArticulationFromJoints(
	JointConstPtr joint1,
	JointConstPtr joint2 )
{
	if ( !joint1 ||
	     !joint2 ||
	     !joint1->IsRevolute() ||
	     !joint2->IsRevolute() )
		return nullptr;

	std::vector< JointConstPtr > maybe_articulation = { joint2, joint1 };
	auto maybe_center = ComputeIntersection( maybe_articulation );

	if ( !maybe_center )
		return nullptr;

	if ( !AxesIndependent( maybe_articulation ) )
		return nullptr;

	return std::shared_ptr< Articulation >( new Articulation(
												ArticulationType::Universal,
												maybe_articulation,
												*maybe_center ) );
}

// ------------------------------------------------------------

std::shared_ptr< Articulation > SkeletonAnalyzer::ExtractArticulationFromJoints(
	JointConstPtr joint1,
	JointConstPtr joint2,
	JointConstPtr joint3 )
{
	if ( !joint1 || !joint2 || !joint3 ||
	     !joint1->IsRevolute() ||
	     !joint2->IsRevolute() ||
	     !joint3->IsRevolute() )
		return nullptr;

	std::vector< JointConstPtr > maybe_articulation = { joint3, joint2, joint1 };
	auto maybe_center = ComputeIntersection( maybe_articulation );

	if ( !maybe_center )
		return nullptr;

	if ( !AxesIndependent( maybe_articulation ) )
		return nullptr;

	return std::shared_ptr< Articulation >( new Articulation(
												ArticulationType::Spherical,
												maybe_articulation,
												*maybe_center ) );
}

// ------------------------------------------------------------

std::shared_ptr< Articulation > SkeletonAnalyzer::ExtractArticulationFromLastJoints(
	const std::span< const JointConstPtr >& joints,
	const Mat4d& tip )
{
	const int n_last_joints = std::min( 3, ( int )joints.size() );
	auto maybe_last_joints = joints.last( n_last_joints );

	if ( n_last_joints == 3 )
	{
		if ( auto articulation = ExtractArticulationFromJoints(
				 maybe_last_joints[2],
				 maybe_last_joints[1],
				 maybe_last_joints[0] ) )
		{
			return articulation;
		}
	}
	if ( n_last_joints >= 2 )
	{
		if ( auto articulation = ExtractArticulationFromJoints(
				 maybe_last_joints[n_last_joints - 1],
				 maybe_last_joints[n_last_joints - 2] ) )
		{
			return articulation;
		}
	}

	auto last_joint = maybe_last_joints[n_last_joints - 1];

	if ( last_joint->IsPrismatic() )
	{
		return std::shared_ptr< Articulation >( new Articulation(
													ArticulationType::Prismatic,
													std::vector< JointConstPtr >{ last_joint },
													last_joint->Origin() ) );
	}

	const Vec3d& p_tip = Translation( tip );
	const Vec3d& tip_dir = p_tip - last_joint->Origin();

	ArticulationType type = last_joint->IsFixed() ? ArticulationType::Fixed : ArticulationType::Revolute;
	Vec3d p_center = last_joint->Origin();

	// Tip on last joint axis
	if ( last_joint->IsFixed() || tip_dir.cross( last_joint->Axis() ).norm() < epsilon )
	{
		p_center = p_tip;
	}

	return std::shared_ptr< Articulation >( new Articulation(
												type,
												std::vector< JointConstPtr >{ last_joint },
												p_center ) );
}

// ------------------------------------------------------------

}