#pragma once

#include "Global.hpp"

#include "ArticulationType.hpp"
#include "Joint.hpp"

#include <memory>
#include <span>

namespace SOArm100::Kinematics::Model
{
class Articulation
{
public:
static std::vector< std::shared_ptr< const Articulation >> ExtractFromJoints(
	const std::span< const JointConstPtr >& joints,
	const Mat4d& tip );

ArticulationType GetType() const {
	return type_;
}

std::span< const JointConstPtr > Joints() const {
	return joints_;
}

int JointCount() const {
	return joints_.size();
}

const Vec3d& Center() const {
	return center_;
}

const Vec3d& Axis() const {
	return ( *joints_.rbegin() )->Axis();
}

private:
ArticulationType type_;
std::vector< JointConstPtr > joints_;
Vec3d center_;

static std::shared_ptr< Articulation > ExtractArticulationFromJoints(
	JointConstPtr joint1 );

static std::shared_ptr< Articulation > ExtractArticulationFromJoints(
	JointConstPtr joint1,
	JointConstPtr joint2 );

static std::shared_ptr< Articulation > ExtractArticulationFromJoints(
	JointConstPtr joint1,
	JointConstPtr joint2,
	JointConstPtr joint3 );

static std::shared_ptr< Articulation > ExtractArticulationFromLastJoints(
	const std::span< const JointConstPtr >& joints,
	const Mat4d& tip );

Articulation(
	ArticulationType type,
	const std::vector< JointConstPtr >& joints,
	const Vec3d& center );
};

using ArticulationConstPtr = std::shared_ptr< const Articulation >;

}