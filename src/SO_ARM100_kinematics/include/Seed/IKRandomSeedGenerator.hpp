#pragma once

#include "Global.hpp"
#include "IKSeedGenerator.hpp"
#include "Model/KinematicModel.hpp"

namespace SOArm100::Kinematics::Seed
{
class IKRandomSeedGenerator : public IKSeedGenerator
{
enum class RandomType
{
    Random,
    Near,
    NearWrapLimit,
    NearCenterLimit
};

struct RandomParameters
{
    double distance {0.1};
    double margin_percent{0.05};
    double min_limit_span{4 * M_PI / 5};
};

public:
IKRandomSeedGenerator( 
    Model::KinematicModelConstPtr model,
    RandomType type, 
    RandomParameters parameters );

virtual VecXd Generate( const Solver::IKProblem& problem ) const override;

private:
Model::KinematicModelConstPtr model_;
RandomType type_;
RandomParameters parameters_;
mutable random_numbers::RandomNumberGenerator rng_;
};
}