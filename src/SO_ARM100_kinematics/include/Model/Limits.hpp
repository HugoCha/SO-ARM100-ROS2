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

void RandomNearWrapped(
	random_numbers::RandomNumberGenerator& rng,
	double seed,
	double* random,
	double distance = 0.05 ) const
{
	if ( random == nullptr )
		throw std::invalid_argument( "random pointer must not be null" );

	const double span = Span();
	const double d = std::min( std::abs( distance ), span / 2.0 );

	double lo = seed - d;
	double hi = seed + d;

	double overflow_min = min_ - lo;
	double overflow_max = hi - max_;

	double seg1_lo = std::max( lo, min_ );
	double seg1_hi = std::min( hi, max_ );

	double seg2_lo = min_;
	double seg2_hi = min_ + std::max( 0.0, overflow_max );

	double seg3_lo = max_ - std::max( 0.0, overflow_min );
	double seg3_hi = max_;

	double len1 = std::max( 0.0, seg1_hi - seg1_lo );
	double len2 = std::max( 0.0, seg2_hi - seg2_lo );
	double len3 = std::max( 0.0, seg3_hi - seg3_lo );
	double total = len1 + len2 + len3;

	if ( total <= 0.0 )
	{
		*random = seed;
		return;
	}

	double pick = rng.uniformReal( 0.0, total );

	if ( pick < len1 )
		*random = seg1_lo + pick;
	else if ( pick < len1 + len2 )
		*random = seg2_lo + ( pick - len1 );
	else
		*random = seg3_lo + ( pick - len1 - len2 );
}

private:
double min_;
double max_;
};
}
