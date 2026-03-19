#pragma once

#include "Model/KinematicModel.hpp"
#include "Seed/IKSeedGenerator.hpp"
#include <vector>

namespace SOArm100::Kinematics::Seed
{
class IKOppositeSeedGenerator : public IKSeedGenerator
{
public:
IKOppositeSeedGenerator( Model::KinematicModelConstPtr model );
IKOppositeSeedGenerator( Model::KinematicModelConstPtr model, const std::span<const int>& joint_indexes );

virtual VecXd Generate( const Solver::IKProblem& problem ) const override;

private:
Model::KinematicModelConstPtr model_;
const std::vector<int> joint_indexes_;

static std::vector<int> EnumerateAllJointIndexes( Model::KinematicModelConstPtr model );

static std::vector<int> UniqueValidJointIndexes( 
    Model::KinematicModelConstPtr model, 
    const std::span<const int> joint_indexes );
};
}