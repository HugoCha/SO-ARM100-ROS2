#include "Seed/IKRandomSeedGenerator.hpp"

#include "Solver/IKProblem.hpp"

namespace SOArm100::Kinematics::Seed
{

// ------------------------------------------------------------

IKRandomSeedGenerator::IKRandomSeedGenerator(
	Model::KinematicModelConstPtr model ) :
	IKRandomSeedGenerator( model, Model::RandomType::Random, RandomParameters() )
{
}

// ------------------------------------------------------------

IKRandomSeedGenerator::IKRandomSeedGenerator(
	Model::KinematicModelConstPtr model,
	Model::RandomType type,
	RandomParameters parameters ) :
	model_( model ),
	type( type ),
	parameters( parameters )
{
}

// ------------------------------------------------------------

VecXd IKRandomSeedGenerator::Generate( const Solver::IKProblem& problem ) const
{
	switch ( type )
	{
	case Model::RandomType::Near:
		return model_->GetChain()->RandomValidJointsNear(
			rng_,
			problem.seed,
			parameters.distance,
			parameters.margin_percent );
	case Model::RandomType::NearWrapLimit:
		return model_->GetChain()->RandomValidJointsNearWrapped(
			rng_,
			problem.seed,
			parameters.min_limit_span,
			parameters.distance,
			parameters.margin_percent );
	case Model::RandomType::NearCenterLimit:
		return model_->GetChain()->RandomValidJointsNearCentered(
			rng_,
			problem.seed,
			parameters.distance,
			parameters.margin_percent );
	default:
	case Model::RandomType::Random:
		return model_->GetChain()->RandomValidJoints(
			rng_,
			parameters.margin_percent );
	}
}

// ------------------------------------------------------------

}