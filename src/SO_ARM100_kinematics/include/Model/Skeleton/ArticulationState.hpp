#pragma once

#include "Global.hpp"

#include "Articulation.hpp"
#include "BoneState.hpp"
#include "Model/Joint/JointState.hpp"

#include <vector>
#include <memory>

namespace SOArm100::Kinematics::Model
{
class ArticulationState
{
public:
ArticulationState( ArticulationConstPtr articulation );
virtual ~ArticulationState() = default;

const Vec3d& Origin() const {
	return center_;
}

const Vec3d& Axis() const {
	return axis_;
}

double Value() const {
	return value_;
}

ArticulationConstPtr GetArticulation() const {
	return articulation_;
}

std::vector< JointStateConstPtr > GetJointStates() const;

VecXd GetJointValues() const;

void SetState( const Quaternion& rotation, const Vec3d& origin, const VecXd& values );
void SetCenterPose( const Quaternion& rotation, const Vec3d& origin );

virtual void ApplyConstraints( BoneState& bone_state ) const = 0;
virtual void UpdateValues( const BoneState& bone_state ) = 0;

protected:
ArticulationConstPtr articulation_;
Quaternion rotation_;
Vec3d center_;
Vec3d axis_;
Iso3d internal_transform_;
double value_;
std::vector< JointStatePtr > joint_states_;
};

using ArticulationStatePtr = std::shared_ptr< ArticulationState >;
using ArticulationStateConstPtr = std::shared_ptr< const ArticulationState >;

}