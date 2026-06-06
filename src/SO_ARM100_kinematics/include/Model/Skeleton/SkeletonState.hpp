#pragma once

#include "Global.hpp"

#include "ArticulationState.hpp"
#include "Model/Skeleton/Articulation.hpp"
#include "BoneState.hpp"
#include "Skeleton.hpp"

#include <map>
#include <span>
#include <vector>

namespace SOArm100::Kinematics::Model
{
class SkeletonState
{
public:
SkeletonState( const Skeleton* skeleton );

const Model::Skeleton* GetSkeleton() const {
	return skeleton_;
}

static constexpr std::map< Model::ArticulationType, double > DampingFactors() 
{
	return {
		{ Model::ArticulationType::Prismatic, 1.0 },
		{ Model::ArticulationType::Revolute,  0.6 },
		{ Model::ArticulationType::Universal, 1.0 },
		{ Model::ArticulationType::Spherical, 1.0 },
	};
};

VecXd GetJointValues() const;

std::span< const ArticulationStatePtr > GetArticulationStates() const {
	return articulation_states_;
}

std::vector< BoneState > GetBoneStates() const;

void SetState( const VecXd& joints );
void ApplyConstraint( BoneState& bone_state, int i ) const;
void UpdateValue( const VecXd& seed, const BoneState& bone_state, int i );

private:
const Model::Skeleton* skeleton_;
std::vector< ArticulationStatePtr > articulation_states_;

VecXd GetArticulationJoints( const VecXd& joints, int i ) const;

static ArticulationStatePtr CreateArticulationState( const Articulation* articulation );
};
}