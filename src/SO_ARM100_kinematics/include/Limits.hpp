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

[[nodiscard]] double Min() const noexcept {
	return min_;
}

[[nodiscard]] double Max() const noexcept {
	return max_;
}

[[nodiscard]] bool Within( double value ) const noexcept {
	return value >= min_ && value <= max_;
}

[[nodiscard]] double Clamp( double value ) const noexcept {
	return std::clamp( value, min_, max_ );
}

void Random( random_numbers::RandomNumberGenerator& rng, double* random ) const {
	if ( random == nullptr )
		throw std::invalid_argument( "random pointer must not be null" );
	*random = rng.uniformReal( min_, max_ );
}

[[nodiscard]] double Range() const noexcept {
	return max_ - min_;
}

private:
double min_;
double max_;
};
}
