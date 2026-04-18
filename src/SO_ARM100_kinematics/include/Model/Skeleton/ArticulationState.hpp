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

const Vec3d& Origin() const {
	return center_;
}

const Vec3d& Axis() const {
	return joint_states_.back()->Axis();
}

double Value() const {
	return joint_states_.back()->Value();
}

ArticulationConstPtr GetArticulation() const {
	return articulation_;
}

std::vector< JointStateConstPtr > GetJointStates() const;

VecXd GetJointValues() const;

void SetState( const Quaternion& rotation, const Vec3d& origin, const VecXd& values );
void SetCenterPose( const Quaternion& rotation, const Vec3d& origin );

void ApplyConstraints( BoneState& bone_state ) const;

void UpdateValue(
	const ArticulationState& old_state,
	const BoneState& old_bone_state,
	const BoneState& new_bone_state );

private:
ArticulationConstPtr articulation_;
Vec3d center_;
Quaternion rotation_;
std::vector< JointStatePtr > joint_states_;

void ApplyPrismaticConstraints( BoneState& bone_state ) const;
void ApplyRevoluteConstraints( BoneState& bone_state ) const;
void ApplyUniversalConstraints( BoneState& bone_state ) const;
void ApplySphericalConstraints( BoneState& bone_state ) const;

void UpdatePrismaticArticulationValue(
	const ArticulationState& old_state,
	const BoneState& old_bone_state,
	const BoneState& new_bone_state );

void UpdateRevoluteArticulationValue(
	const ArticulationState& old_state,
	const BoneState& old_bone_state,
	const BoneState& new_bone_state );

void UpdateUniversalArticulationValue(
	const ArticulationState& old_state,
	const BoneState& old_bone_state,
	const BoneState& new_bone_state );

void UpdateSphericalArticulationValue(
	const ArticulationState& old_state,
	const BoneState& old_bone_state,
	const BoneState& new_bone_state );
};

using ArticulationStatePtr = std::shared_ptr< ArticulationState >;
using ArticulationStateConstPtr = std::shared_ptr< const ArticulationState >;

}