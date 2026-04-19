#pragma once

#include "Global.hpp"

#include "Model/Geometry/Pose.hpp"

#include <memory>

namespace SOArm100::Kinematics::Model
{
class JointState
{
public:
JointState( const JointConstPtr& joint );

const double& Value() const {
	return value_;
}

double& Value(){
	return value_;
}

const Vec3d& Origin() const {
	return pose_.origin;
}

const Vec3d& Axis() const {
	return pose_.axis;
}

bool UpdateValueRequired() const {
	return update_value_required_;
}

bool UpdatePoseRequired() const {
	return update_pose_required_;
}

void SetState( const Vec3d& origin, const Vec3d& axis, double value );
void SetPose( const Vec3d& origin, const Vec3d& axis );

void UpdatePose(
	const Vec3d& origin,
	const Vec3d& axis );

void UpdateValue(
	const Vec3d& old_origin,
	const Vec3d& old_to_child_direction,
	const Vec3d& new_to_child_direction );

private:
bool update_value_required_;
bool update_pose_required_;
Pose pose_;
double value_;
JointConstPtr joint_;

void UpdatePrismaticValue(
	const Vec3d& old_orgin );

void UpdateRevoluteValue(
	const Vec3d& old_to_child_direction,
	const Vec3d& new_to_child_direction );
};

using JointStatePtr = std::shared_ptr< JointState >;
using JointStateConstPtr = std::shared_ptr< const JointState >;

}