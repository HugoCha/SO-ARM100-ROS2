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
ArticulationState( const Articulation* articulation );
virtual ~ArticulationState() = default;

const Iso3d GlobalTransform() const {
	return global_transform_;
}

const Iso3d& LocalTransform() const {
	return local_transform_;
}

const Articulation* GetArticulation() const {
	return articulation_;
}

std::vector< JointStateConstPtr > GetJointStates() const;

VecXd GetJointValues() const;

void SetState( const Iso3d& world_transform, const VecXd& values );
void SetCenterPose( const Iso3d& world_transform );

virtual void ApplyConstraints( BoneState& bone_state ) const = 0;
virtual void UpdateValues( const BoneState& bone_state ) = 0;

protected:
const Articulation* articulation_;
Iso3d world_transform_;
Iso3d global_transform_;
Iso3d local_transform_;

std::vector< JointStatePtr > joint_states_;

void SetJointInternalState(
	JointStatePtr& joint_state,
	const Iso3d& world_transform,
	Iso3d& global_transform,
	Iso3d& local_transform,
	double value ) const;
};

using ArticulationStatePtr = std::shared_ptr< ArticulationState >;
using ArticulationStateConstPtr = std::shared_ptr< const ArticulationState >;

}