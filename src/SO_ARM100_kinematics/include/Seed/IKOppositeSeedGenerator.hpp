#pragma once

#include "Model/KinematicModel.hpp"
#include "Seed/IKSeedGenerator.hpp"

#include <set>

namespace SOArm100::Kinematics::Seed
{
class IKOppositeSeedGenerator : public IKSeedGenerator
{
public:
IKOppositeSeedGenerator( Model::KinematicModelConstPtr model );
IKOppositeSeedGenerator( Model::KinematicModelConstPtr model, Model::JointGroup group );
IKOppositeSeedGenerator( Model::KinematicModelConstPtr model, const std::set< int >& joint_indexes );

virtual VecXd Generate( const Solver::IKProblem& problem ) const override;

private:
Model::KinematicModelConstPtr model_;
const std::set< int > joint_indexes_;

static std::set< int > EnumerateAllJointIndexes( Model::KinematicModelConstPtr model );
};
}