#pragma once

#include "Global.hpp"

#include "ArticulationState.hpp"
#include "Model/Skeleton/Articulation.hpp"
#include "BoneState.hpp"
#include "Skeleton.hpp"

#include <span>
#include <vector>

namespace SOArm100::Kinematics::Model
{
class SkeletonState
{
public:
SkeletonState( const Skeleton* skeleton );

VecXd GetJointValues() const;

std::span< const ArticulationStatePtr > GetArticulationStates() const {
	return articulation_states_;
}

std::vector< BoneState > GetBoneStates() const;

void SetState( const VecXd& joints );
void ApplyConstraint( BoneState& bone_state, int i ) const;
void UpdateValue( const BoneState& bone_state, int i );

private:
const Model::Skeleton* skeleton_;
std::vector< ArticulationStatePtr > articulation_states_;

static ArticulationStatePtr CreateArticulationState( const Articulation* articulation );
};
}