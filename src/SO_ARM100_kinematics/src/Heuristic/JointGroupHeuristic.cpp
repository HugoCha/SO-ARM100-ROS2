#include "Heuristic/JointGroupHeuristic.hpp"

#include <stdexcept>

namespace SOArm100::Kinematics::Heuristic
{

// ------------------------------------------------------------

JointGroupHeuristic::JointGroupHeuristic(
	Model::KinematicModelConstPtr model,
	const Model::JointGroup& group ) :
	IKHeuristic( model ),
	group_( group ),
	ancestor_( std::nullopt ),
	successor_( std::nullopt )
{
	if ( Model::JointGroup::IsConsistent( *model->GetChain(), group ) )
	{
		ancestor_ = ComputeAncestorJointGroup( model, group );
		successor_ = ComputeSuccessorJointGroup( model, group );
	}
	else
	{
		throw std::invalid_argument( "Invalid group" );
	}
}

// ------------------------------------------------------------

Mat4d JointGroupHeuristic::ComputeGroupFirstJointPose( const VecXd& seed ) const
{
	const auto& first_joint_origin =
		GetChain()->GetActiveJoint( group_.FirstIndex() )->OriginTransform();

	if ( !ancestor_ )
		return first_joint_origin;

	Mat4d T_first_joint;
	VecXd ancestor_joints = ancestor_->GetGroupJoints( seed );

	GetChain()->ComputeFK(
		ancestor_joints,
		first_joint_origin,
		T_first_joint );

	return T_first_joint;
}

// ------------------------------------------------------------

Mat4d JointGroupHeuristic::ComputeGroupTarget(
	const VecXd& seed,
	const Mat4d& target ) const
{
	Mat4d T_tip;

	if ( ancestor_ )
	{
		VecXd ancestor_joints = ancestor_->GetGroupJoints( seed );
		GetChain()->ComputeFK(
			ancestor_joints,
			group_.tip_home,
			T_tip );
	}
	else
	{
		T_tip = group_.tip_home;
	}

	return target * Inverse( T_tip );
}

// ------------------------------------------------------------

std::optional< Model::JointGroup > JointGroupHeuristic::ComputeAncestorJointGroup(
	Model::KinematicModelConstPtr model,
	const Model::JointGroup& group )
{
	int ancestor_count = group.FirstIndex();

	if ( ancestor_count == 0 )
		return std::nullopt;

	Mat4d ancestor_tip = model->GetChain()->GetActiveJoint( ancestor_count )->OriginTransform();

	return Model::JointGroup::CreateFromRange(
		"ancestor",
		0,
		ancestor_count,
		ancestor_tip );
}

// ------------------------------------------------------------

std::optional< Model::JointGroup > JointGroupHeuristic::ComputeSuccessorJointGroup(
	Model::KinematicModelConstPtr model,
	const Model::JointGroup& group )
{
	int sucessor_start = group.LastIndex();
	int sucessor_count = model->GetChain()->GetActiveJointCount() - sucessor_start;

	if ( sucessor_count == 0 )
		return std::nullopt;

	Mat4d successor_tip = model->GetHomeConfiguration();

	return Model::JointGroup::CreateFromRange(
		"successor",
		sucessor_start,
		sucessor_count,
		successor_tip );
}

// ------------------------------------------------------------

}