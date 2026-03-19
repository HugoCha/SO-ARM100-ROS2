#pragma once

#include "Global.hpp"

#include "IKHeuristicState.hpp"
 
namespace SOArm100::Kinematics::Heuristic
{
struct IKPresolution
{
VecXd joints;
IKHeuristicState state;

bool Sucess() const {
    return state == IKHeuristicState::Success;
}

bool PartialOrSuccess() const {
    return state == IKHeuristicState::PartialSuccess || state == IKHeuristicState::Success;
}

bool Fail() const {
    return state == IKHeuristicState::Fail;
}
};
}