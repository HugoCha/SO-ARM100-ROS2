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

Vec3d& Origin() {
	return pose_.origin;
}

const Vec3d& Axis() const {
	return pose_.axis;
}

Vec3d& Axis() {
	return pose_.axis;
}

private:
Pose pose_;
double value_;
JointConstPtr joint_;
};

using JointStatePtr = std::shared_ptr< JointState >;
using JointStateConstPtr = std::shared_ptr< const JointState >;

}