#pragma once

#include "WeightedScorer.hpp"

namespace SOArm100::Kinematics::Scorer
{
class WeightedScorerBuilder {
public:
WeightedScorerBuilder& Add( double weight, std::unique_ptr< const IKSolutionScorer > scorer )
{
	scorers_.emplace_back( weight, std::move( scorer ) );
	return *this;
}

std::unique_ptr< WeightedScorer > Build()
{
	return std::make_unique< WeightedScorer >( std::move( scorers_ ) );
}

private:
std::vector< WeightedScorer::WeightScorerPair > scorers_;
};
}