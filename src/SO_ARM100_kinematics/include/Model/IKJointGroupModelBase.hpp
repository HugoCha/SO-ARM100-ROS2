#pragma once

#include "Model/IKModelBase.hpp"

namespace SOArm100::Kinematics::Model
{
class IKJointGroupModelBase : public IKModelBase
{
public:
IKJointGroupModelBase(
	Model::KinematicModelConstPtr model,
	const Model::JointGroup& group );

Model::JointGroup GetGroup() const {
	return group_;
}

std::optional< Model::JointGroup > GetAncestor() const {
	return ancestor_;
}

std::optional< Model::JointGroup > GetSuccessor() const {
	return successor_;
}

protected:
Mat4d ComputeGroupFirstJointPose( const VecXd& seed ) const;
Mat4d ComputeGroupTarget( const VecXd& seed, const Mat4d& target ) const;

private:
std::optional< Model::JointGroup > ancestor_;
Model::JointGroup group_;
std::optional< Model::JointGroup > successor_;
Mat4d home_in_tip_inv_;

static std::optional< Model::JointGroup > ComputeAncestorJointGroup(
	Model::KinematicModelConstPtr model,
	const Model::JointGroup& group );

static std::optional< Model::JointGroup > ComputeSuccessorJointGroup(
	Model::KinematicModelConstPtr model,
	const Model::JointGroup& group );

static Mat4d ComputeHomeInTipTransform(
	Model::KinematicModelConstPtr model,
	const Model::JointGroup& group );
};
}