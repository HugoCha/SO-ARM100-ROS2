#pragma once

#include "WeightedScorers.hpp"

namespace SOArm100::Kinematics::Scorer
{
class WeightedScorersBuilder {
public:
WeightedScorersBuilder& Add( double weight, std::unique_ptr< const IKSolutionScorer > scorer )
{
	scorers_.emplace_back( weight, std::move( scorer ) );
	return *this;
}

std::unique_ptr< WeightedScorers > Build()
{
	return std::make_unique< WeightedScorers >( std::move( scorers_ ) );
}

private:
std::vector< WeightedScorers::WeightScorerPair > scorers_;
};
}