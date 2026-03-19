#include "Seed/IKRandomSeedGenerator.hpp"

#include "Solver/IKProblem.hpp"

namespace SOArm100::Kinematics::Seed
{

// ------------------------------------------------------------

IKRandomSeedGenerator::IKRandomSeedGenerator( 
    Model::KinematicModelConstPtr model,
    RandomType type, 
    RandomParameters parameters ) :
    model_( model ),
    type_( type ),
    parameters_( parameters )
{
}

// ------------------------------------------------------------

VecXd IKRandomSeedGenerator::Generate( const Solver::IKProblem& problem ) const
{
    switch ( type_ ) 
    {
        case RandomType::Near:
        return model_->GetChain()->RandomValidJointsNear( 
            rng_, 
            problem.seed, 
            parameters_.distance, 
            parameters_.margin_percent );
        case RandomType::NearWrapLimit:
            return model_->GetChain()->RandomValidJointsNearWrapped(
                rng_, 
                problem.seed, 
                parameters_.min_limit_span, 
                parameters_.distance,
                parameters_.margin_percent );
        case RandomType::NearCenterLimit:
            return model_->GetChain()->RandomValidJointsNearCentered( 
                rng_, 
                problem.seed, 
                parameters_.distance, 
                parameters_.margin_percent );
        default:
        case RandomType::Random:
            return model_->GetChain()->RandomValidJoints( 
                rng_, 
                parameters_.margin_percent );
    }
}

// ------------------------------------------------------------

}