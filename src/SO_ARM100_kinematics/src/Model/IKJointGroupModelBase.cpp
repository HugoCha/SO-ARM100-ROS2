#include "Model/IKJointGroupModelBase.hpp"

#include "Global.hpp"
#include "Model/IKModelBase.hpp"
#include "Utils/KinematicsUtils.hpp"

#include <stdexcept>

namespace SOArm100::Kinematics::Model
{

// ------------------------------------------------------------

IKJointGroupModelBase::IKJointGroupModelBase(
	Model::KinematicModelConstPtr model,
	const Model::JointGroup& group ) :
	IKModelBase( model ),
	group_( group ),
	ancestor_( std::nullopt ),
	successor_( std::nullopt )
{
	if ( Model::JointGroup::IsConsistent( *model->GetChain(), group ) )
	{
		ancestor_ = ComputeAncestorJointGroup( model, group );
		successor_ = ComputeSuccessorJointGroup( model, group );
		auto home_in_tip = ComputeHomeInTipTransform( model, group );
		home_in_tip_inv_ = Inverse( home_in_tip );
	}
	else
	{
		throw std::invalid_argument( "Invalid group" );
	}
}

// ------------------------------------------------------------

Mat4d IKJointGroupModelBase::ComputeGroupFirstJointPose( const VecXd& seed ) const
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

Mat4d IKJointGroupModelBase::ComputeGroupTarget(
	const VecXd& seed,
	const Mat4d& target ) const
{
	if ( !ancestor_ && !successor_ )
		return target;

	Mat4d T_ancestor_inv;
	
	if ( ancestor_ )
	{
		Mat4d T_ancestor;

		VecXd ancestor_joints = ancestor_->GetGroupJoints( seed );
		GetChain()->ComputeFK(
			ancestor_joints,
			group_.tip_home,
			T_ancestor );

		T_ancestor_inv = Inverse( T_ancestor );

		if ( !successor_ )
			return T_ancestor_inv * target;
	}

	Mat4d T_successor_inv;
	T_successor_inv = home_in_tip_inv_;

	if ( !ancestor_ )
		return target * T_successor_inv;

	return T_ancestor_inv * target * T_successor_inv;
}

// ------------------------------------------------------------

std::optional< Model::JointGroup > IKJointGroupModelBase::ComputeAncestorJointGroup(
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

std::optional< Model::JointGroup > IKJointGroupModelBase::ComputeSuccessorJointGroup(
	Model::KinematicModelConstPtr model,
	const Model::JointGroup& group )
{
	int sucessor_start = group.LastIndex() + 1;
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

Mat4d IKJointGroupModelBase::ComputeHomeInTipTransform( 
	Model::KinematicModelConstPtr model, 
	const Model::JointGroup& group )
{
	return model->GetHomeConfiguration() * Inverse( group.tip_home );
}

// ------------------------------------------------------------

}