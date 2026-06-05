#pragma once

#include "Global.hpp"
#include "IKModelBase.hpp"
#include "Model/Joint/JointGroup.hpp"

namespace SOArm100::Kinematics::Model
{
class IKJointGroupModelBase : public IKModelBase
{
public:
IKJointGroupModelBase(
	Model::KinematicModelConstPtr model,
	const Model::JointGroup& group );

const Model::JointGroup& GetGroup() const {
	return group_;
}

std::optional< Model::JointGroup > GetAncestor() const {
	return ancestor_;
}

std::optional< Model::JointGroup > GetSuccessor() const {
	return successor_;
}

double GroupMaxReach() const {
	return group_max_reach_;
}

Mat4d ComputeGroupLocalTarget( const VecXd& seed, const Mat4d& target ) const;
Mat4d ComputeGroupWorldTarget( const VecXd& seed, const Mat4d& target ) const;

bool ComputeGroupLocalJointStatesFK( const VecXd& joints, std::vector< Model::JointState >& joint_states, Mat4d& fk ) const {
	return ComputeGroupJointStatesFK( 
		joints, 
		Inverse( GetActiveJoint( GetGroup().FirstIndex() )->OriginTransform() ), 
		joint_states, 
		fk );
}

bool ComputeGroupLocalJointPosesFK( const VecXd& joints, std::vector< Mat4d >& joint_poses, Mat4d& fk ) const {
	return ComputeGroupJointPosesFK( 
		joints, 
		Inverse( GetActiveJoint( GetGroup().FirstIndex() )->OriginTransform() ),
		joint_poses, 
		fk );
}

bool ComputeGroupLocalFK( const VecXd& joints, Mat4d& fk ) const {
	return ComputeGroupFK( 
		joints, 
		Inverse( GetActiveJoint( GetGroup().FirstIndex() )->OriginTransform() ),
		fk );
}

bool ComputeGroupWorldJointStatesFK( const VecXd& joints, std::vector< Model::JointState >& joint_states, Mat4d& fk ) const {
	return ComputeGroupJointStatesFK( 
		joints, 
		GetAncestorTransform( joints ), 
		joint_states, 
		fk );
}
bool ComputeGroupWorldJointPosesFK( const VecXd& joints, std::vector< Mat4d >& joint_poses, Mat4d& fk ) const {
	return ComputeGroupJointPosesFK( 
		joints, 
		GetAncestorTransform( joints ), 
		joint_poses, fk );
}

bool ComputeGroupWorldFK( const VecXd& joints, Mat4d& fk ) const {
	return ComputeGroupFK( joints, Mat4d::Identity(), fk );
}

Mat4d GetAncestorTransform( const VecXd& seed ) const;
Mat4d GetAncestorInverseTransform( const VecXd& seed ) const;
Mat4d GetSuccessorInverseTransform() const {
	return home_in_tip_inv_;
}

private:
std::optional< Model::JointGroup > ancestor_;
Model::JointGroup group_;
std::optional< Model::JointGroup > successor_;
Mat4d home_in_tip_inv_;
double group_max_reach_;

static std::optional< Model::JointGroup > ComputeAncestorJointGroup(
	Model::KinematicModelConstPtr model,
	const Model::JointGroup& group );

static std::optional< Model::JointGroup > ComputeSuccessorJointGroup(
	Model::KinematicModelConstPtr model,
	const Model::JointGroup& group );

static Mat4d ComputeHomeInTipTransform(
	Model::KinematicModelConstPtr model,
	const Model::JointGroup& group );

static double ComputeGroupMaxReach( 	
	const KinematicModelConstPtr& model, 
	const JointGroup& group );

bool ComputeGroupJointStatesFK(
	const VecXd& joints,
	const Mat4d& initial_transform,
	std::vector< Model::JointState >& joint_states,
	Mat4d& fk ) const;

bool ComputeGroupJointPosesFK(
	const VecXd& joints,
	const Mat4d& initial_transform,
	std::vector< Mat4d >& joint_poses,
	Mat4d& fk ) const;

bool ComputeGroupFK(
	const VecXd& joints,
	const Mat4d& initial_transform,
	Mat4d& fk ) const;
};
}