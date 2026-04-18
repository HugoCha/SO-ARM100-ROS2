#include "Model/IKJointGroupModelBase.hpp"

#include "Global.hpp"

#include "Model/IKModelBase.hpp"
#include "Model/Joint/JointState.hpp"
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

Mat4d IKJointGroupModelBase::ComputeGroupLocalTarget(
	const VecXd& seed,
	const Mat4d& target ) const
{
	Mat4d T_target = target;

	if ( ancestor_ )
	{
		Mat4d T_ancestor_inv = GetAncestorInverseTransform( seed );
		T_target = T_ancestor_inv * T_target;
	}
	if ( successor_ )
	{
		Mat4d T_successor_inv = GetSuccessorInverseTransform();
		T_target = T_target * T_successor_inv;
	}

	return T_target;
}

// ------------------------------------------------------------

Mat4d IKJointGroupModelBase::ComputeGroupWorldTarget(
	const VecXd& seed,
	const Mat4d& target ) const
{
	Mat4d T_target = target;

	if ( successor_ )
	{
		Mat4d T_successor_inv = GetSuccessorInverseTransform();
		T_target = T_target * T_successor_inv;
	}

	return T_target;
}

// ------------------------------------------------------------

bool IKJointGroupModelBase::ComputeGroupJointStatesFK(
	const VecXd& joints,
	const Mat4d& to_local_transform,
	std::vector< Model::JointState >& states,
	Mat4d& fk ) const
{
	// const int n_joints = joints.size();
	// const int n_model_joints = GetChain()->GetActiveJointCount();
	// const int n_group_joints = GetGroup().Size();

	// if ( states.size() < n_group_joints )
	// 	states.resize( GetGroup().Size() );

	// if ( n_joints != n_model_joints )
	// 	throw std::invalid_argument( "Joints size mismatch" );

	// if ( !GetChain()->WithinLimits( joints.topRows( GetGroup().LastIndex() + 1 ) ) )
	// 	return false;

	// Mat4d T_cumul = to_local_transform;

	// int state_index = 0;
	// for ( int i = GetGroup().FirstIndex(); i <= GetGroup().LastIndex(); i++ )
	// {
	// 	const auto& joint = GetActiveJoint( i );

	// 	if ( GetGroup().indices.contains( i ) )
	// 	{
	// 		states[state_index].pose  = joint->Pose( T_cumul );
	// 		states[state_index].value = joints[i];
	// 		state_index++;
	// 	}

	// 	const auto& twist = joint->GetTwist();
	// 	T_cumul *= twist.ExponentialMatrix( joints[i] );
	// }

	// fk.noalias() = T_cumul * GetGroup().tip_home;
	// return true;
	return false;
}

// ------------------------------------------------------------

bool IKJointGroupModelBase::ComputeGroupFK(
	const VecXd& joints,
	const Mat4d& to_local_transform,
	Mat4d& fk ) const
{
	const int n_joints = joints.size();
	const int n_model_joints = GetChain()->GetActiveJointCount();
	const int n_group_joints = GetGroup().Size();

	if ( n_joints != n_model_joints )
		throw std::invalid_argument( "Joints size mismatch" );

	if ( !GetChain()->WithinLimits( joints.topRows( GetGroup().LastIndex() + 1 ) ) )
		return false;

	Mat4d T_cumul = to_local_transform;

	int state_index = 0;
	for ( int i = GetGroup().FirstIndex(); i <= GetGroup().LastIndex(); i++ )
	{
		const auto& joint = GetActiveJoint( i );
		const auto& twist = joint->GetTwist();
		T_cumul *= twist.ExponentialMatrix( joints[i] );
	}

	fk.noalias() = T_cumul * GetGroup().tip_home;
	return true;
}

// ------------------------------------------------------------

bool IKJointGroupModelBase::ComputeGroupJointPosesFK(
	const VecXd& joints,
	const Mat4d& to_local_transform,
	std::vector< Mat4d >& joint_poses,
	Mat4d& fk ) const
{
	const int n_joints = joints.size();
	const int n_model_joints = GetChain()->GetActiveJointCount();
	const int n_group_joints = GetGroup().Size();

	if ( joint_poses.size() < n_group_joints )
		joint_poses.resize( GetGroup().Size() );

	if ( n_joints != n_model_joints )
		throw std::invalid_argument( "Joints size mismatch" );

	if ( !GetChain()->WithinLimits( joints.topRows( GetGroup().LastIndex() + 1 ) ) )
		return false;

	Mat4d T_cumul = to_local_transform;

	int pose_index = 0;
	for ( int i = GetGroup().FirstIndex(); i <= GetGroup().LastIndex(); i++ )
	{
		const auto& joint = GetActiveJoint( i );

		if ( GetGroup().indices.contains( i ) )
		{
			joint_poses[i].noalias() = T_cumul * joint->OriginTransform();
			pose_index++;
		}

		const auto& twist = joint->GetTwist();
		T_cumul *= twist.ExponentialMatrix( joints[i] );
	}

	fk.noalias() = T_cumul * GetGroup().tip_home;
	return true;
}

// ------------------------------------------------------------

Mat4d IKJointGroupModelBase::GetAncestorInverseTransform( const VecXd& seed ) const
{
	if ( !ancestor_ )
		return Mat4d::Identity();

	Mat4d T_ancestor;

	VecXd ancestor_joints = ancestor_->GetGroupJoints( seed );
	GetChain()->ComputeFK(
		ancestor_joints,
		ancestor_->tip_home,
		T_ancestor );

	return Inverse( T_ancestor );
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