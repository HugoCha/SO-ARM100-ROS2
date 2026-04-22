#pragma once

#include "Global.hpp"

#include "ArticulationState.hpp"
#include "Model/Skeleton/Articulation.hpp"
#include "Skeleton.hpp"

#include <vector>

namespace SOArm100::Kinematics::Model
{
class ArticulationState;

class SkeletonState
{
public:
SkeletonState( SkeletonConstPtr skeleton );

VecXd GetJointValues() const;

void SetState( const VecXd& joints );
bool RefreshRequired() const;
void Refresh();

private:
SkeletonConstPtr skeleton_;
std::vector< ArticulationStatePtr > articulation_states_;

static ArticulationStatePtr CreateArticulationState( const ArticulationConstPtr& articulation );
};
}