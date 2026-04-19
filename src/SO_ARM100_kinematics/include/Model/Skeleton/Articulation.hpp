#pragma once

#include "Global.hpp"

#include "ArticulationType.hpp"
#include "Model/Joint/Joint.hpp"

#include <memory>
#include <span>

namespace SOArm100::Kinematics::Model
{
class Articulation
{
public:
Articulation(
	ArticulationType type,
	const std::vector< JointConstPtr >& joints,
	const Vec3d& center ) :
	type_( type ),
	joints_( joints ),
	center_( center )
{
}

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


bool IsEmpty() const {
	return joints_.empty();
}

private:
ArticulationType type_;
std::vector< JointConstPtr > joints_;
Vec3d center_;
};

using ArticulationConstPtr = std::shared_ptr< const Articulation >;

}