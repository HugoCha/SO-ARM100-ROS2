#pragma once

#include <algorithm>
#include <cassert>
#include <limits>
#include <random_numbers/random_numbers.h>

namespace SOArm100::Kinematics
{
class Limits {
public:
Limits() :
	Limits( -std::numeric_limits< double >::infinity(),
	        std::numeric_limits< double >::infinity() )
{
}

Limits( double min, double max ) :
	min_( min ),
	max_( max )
{
	if ( min > max )
	{
		throw std::invalid_argument( "min must be less than or equal to max" );
	}
}

[[nodiscard]] double Center() const noexcept {
	return ( max_ + min_ ) / 2.0;
}

[[nodiscard]] double Min() const noexcept {
	return min_;
}

[[nodiscard]] double Max() const noexcept {
	return max_;
}

[[nodiscard]] double Span() const noexcept {
	return max_ - min_;
}

[[nodiscard]] bool Within( double value ) const noexcept {
	return value >= min_ && value <= max_;
}

[[nodiscard]] double Clamp( double value ) const noexcept {
	return std::clamp( value, min_, max_ );
}

void Random( random_numbers::RandomNumberGenerator& rng, 
			 double* random, 
			 double margin_percent = 0.0 ) const {
	if ( random == nullptr )
		throw std::invalid_argument( "random pointer must not be null" );
	margin_percent = std::clamp( margin_percent, 0.0, 1.0 );
	double margin = margin_percent * Span();
	double min = min_ + margin;
	double max = max_ - margin;
	*random = rng.uniformReal( min, max );
}

void RandomNear( random_numbers::RandomNumberGenerator& rng, 
				 double seed, 
				 double* random, 
				 double distance = 0.05, 
				 double margin_percent = 0.0 ) const {
	if ( random == nullptr )
		throw std::invalid_argument( "random pointer must not be null" );
	margin_percent = std::clamp( margin_percent, 0.0, 1.0 );
	double margin = margin_percent * Span();
	double min = std::max( min_ + margin, seed - std::abs( distance ) );
	double max = std::min( max_ - margin, seed + std::abs( distance ) );
	*random = rng.uniformReal( min, max );
}

[[nodiscard]] double Range() const noexcept {
	return max_ - min_;
}

private:
double min_;
double max_;
};
}
