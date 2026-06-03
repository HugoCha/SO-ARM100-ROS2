#include "Model/IKJointGroupModelBase.hpp"

#include "Global.hpp"

#include "Model/IKModelBase.hpp"
#include "Model/Joint/JointState.hpp"
#include "Utils/KinematicsUtils.hpp"
#include "Utils/Distance.hpp"

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
		group_max_reach_ = ComputeGroupMaxReach( model, group );
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
	const int n_joints = joints.size();
	const int n_model_joints = GetChain()->GetActiveJointCount();
	const int n_group_joints = GetGroup().Size();

	if ( states.size() < n_group_joints )
		throw std::invalid_argument( "Joints state size mismatch" );

	if ( n_joints != n_model_joints )
		throw std::invalid_argument( "Joints size mismatch" );

	if ( !GetChain()->WithinLimits( joints.topRows( GetGroup().LastIndex() + 1 ) ) )
		return false;

	Mat4d T_cumul = to_local_transform;

	int state_index = 0;
	for ( int i = GetGroup().FirstIndex(); i <= GetGroup().LastIndex(); i++ )
	{
		const auto& joint = GetActiveJoint( i );

		if ( GetGroup().indices.contains( i ) )
		{
			auto joint_pose = joint->Pose( T_cumul );
			states[state_index].Origin()  = joint_pose.origin;
			states[state_index].Axis()  = joint_pose.axis;
			states[state_index].Value() = joints[i];
			state_index++;
		}

		const auto& twist = joint->GetTwist();
		T_cumul *= twist.ExponentialMatrix( joints[i] );
	}

	fk.noalias() = T_cumul * GetGroup().tip_home;
	return true;
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

double IKJointGroupModelBase::ComputeGroupMaxReach( 
	const KinematicModelConstPtr& model, 
	const JointGroup& group )
{
    double max_reach = 0.0;
	const auto& chain = model->GetChain();
    for ( int i = 0; i < group.Size() - 1; i++ )
    {
        auto joint = chain->GetActiveJoint( group.Index( i ) );
        auto next_joint = chain->GetActiveJoint( group.Index( i + 1 ) );
        max_reach += Utils::Distance( joint->Origin(), next_joint->Origin() );
    }

    auto last_joint = chain->GetActiveJoint( group.LastIndex() );
    max_reach += Utils::Distance( 
        last_joint->Origin(), 
        Translation( group.tip_home ) );

	return max_reach;
}

// ------------------------------------------------------------

}